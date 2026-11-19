// SPDX-License-Identifier: GPL-2.0
/*
 * MT6351 Battery Driver via internal AUXADC
 * Based on preloader PMIC_IMM_GetOneChannelValue()
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/ktime.h>
#include <linux/math64.h>

/* MT6351 AUXADC registers (from upmu_hw.h) */
#define MT6351_TOP_CKPDN_CON0          0x023A
#define MT6351_TOP_CKPDN_CON1          0x0240
#define MT6351_TOP_CKHWEN_CON0_CLR     0x0286
#define MT6351_AUXADC_CON0             0x0EA2  /* AUXADC control (CK_AON bit15) — было 0x0E00 (ошибка: это регистр результата ADC0) */
#define MT6351_AUXADC_RQST0_SET        0x0E98
#define MT6351_AUXADC_ADC23            0x0E2E  /* CH0_BY_AP (BATSNS) результат — было 0x0C2E (чужой блок, застывший мусор) */

/* Ёмкость батареи (мкАч) — у M5 Note 4000 мАч */
#define MT6351_BAT_CHARGE_FULL	((s64)charge_full_mah * 1000)	/* мкА·ч из мА·ч */

/* Field definitions */
#define MT6351_AUXADC_CK_AON_SHIFT         15
#define MT6351_RG_AUXADC_SMPS_CK_PDN_SHIFT 9
#define MT6351_RG_AUXADC_CK_PDN_SHIFT      10
#define MT6351_AUXADC_RQST_CH0             BIT(0)
#define MT6351_AUXADC_ADC_OUT_MASK         0x7FFF
#define MT6351_AUXADC_ADC_RDY_SHIFT        15
#define MT6351_POLL_INTERVAL	(10 * HZ)

/* Conversion: (raw * R_VAL * V_FULL) / 32768 — vendor: mV = raw × 3 × 1800 / 32768 (BY_AP-каналы 15-бит) */
#define VOLTAGE_FULL_RANGE  1800
#define R_VAL_TEMP_CH0      3
#define MT6351_BAT_SAMPLES 8

/* FGADC (fuel gauge): vendor-механика из battery_meter_hal.c (MT6351)
 * CON0=0x0CA4: бит10 LATCHDATA_ST (статус), старший байт = команда (0x02 READ,
 * 0x08 SW-clear); результат FG_CURRENT_OUT=CON11=0x0CBA, signed 16-bit.
 * Масштаб: UNIT_FGCURRENT=158122 -> uA = counts * 158.122 * (20 / R_FG_мОм).
 * Кросс-чеки 15.11 (мА против bq12/бюджета системы) дали R_FG ≈ 10 мОм. */
#define MT6351_FGADC_CON0          0x0CA4
#define MT6351_FGADC_CON11         0x0CBA
#define MT6351_FG_LATCHDATA_ST     BIT(10)
#define MT6351_FG_UNIT_UA_x1000    158122ULL /* 158.122 uA на count, x1000 */
#define MT6351_FG_OCV_REST_UA      5000      /* |I| < 5 мА -> считаем покоем */
#define MT6351_FG_OCV_REST_POLLS   3         /* покой подряд перед OCV-якорем */
#define MT6351_FG_FULL_ANCHOR_UA   150000    /* FULL-якорь только при |I| < 150 мА */

struct mt6351_bat {
    struct device *dev;
    struct regmap *regmap;
    struct power_supply *psy;
    struct power_supply *charger;   /* ссылка на bq25890 */
    struct mutex lock;
    bool init_done;
    int cached_mv;      /* последнее прочитанное напряжение */
    int cached_pct;
    int last_status;                /* для отслеживания изменений */
    /* кулон-счёт (SOC) */
    s64 charge_uah;     /* расчётный заряд, мкА·ч */
    int fg_ua;          /* последний ток FGADC, мкА (+ = заряд) */
    int last_pct;
    int rest_count;     /* подряд идущие "покойные" опросы */
    int full_count;     /* подряд идущие подтверждения терминации */
    ktime_t last_ts;
    struct delayed_work poll_work;
};

/* R_FG (шунт платы) в мОм — переопределяется без пересборки */
static int fg_r_mohm = 10;
module_param(fg_r_mohm, int, 0644);
MODULE_PARM_DESC(fg_r_mohm, "FGADC sense resistor, mOhm (default 10)");

/* фактическая ёмкость банки, мА·ч — для SOC; калибруется на живой батарее */
static int charge_full_mah = 4000;
module_param(charge_full_mah, int, 0644);
MODULE_PARM_DESC(charge_full_mah, "battery full capacity, mAh (default 4000)");

/* Таблица разряда литий-ионной батареи (напряжение → процент) */
struct v2p_entry {
    int mv;
    int pct;
};

static const struct v2p_entry v2p_table[] = {
    { 4350, 100 },
    { 4280,  95 },
    { 4210,  90 },
    { 4150,  85 },
    { 4100,  80 },
    { 4050,  75 },
    { 4000,  70 },
    { 3960,  65 },
    { 3920,  60 },
    { 3885,  55 },
    { 3855,  50 },
    { 3830,  45 },
    { 3805,  40 },
    { 3785,  35 },
    { 3760,  30 },
    { 3735,  25 },
    { 3705,  20 },
    { 3660,  15 },
    { 3600,  10 },
    { 3500,   5 },
    { 3400,   0 },
};

static int mt6351_v2p(int mv)
{
    int i, n = ARRAY_SIZE(v2p_table);

    if (mv >= v2p_table[0].mv)
        return 100;
    if (mv <= v2p_table[n - 1].mv)
        return 0;

    for (i = 0; i < n - 1; i++) {
        if (mv <= v2p_table[i].mv && mv > v2p_table[i + 1].mv) {
            int dv = v2p_table[i].mv - v2p_table[i + 1].mv;
            int dp = v2p_table[i].pct - v2p_table[i + 1].pct;
            return v2p_table[i].pct -
            ((v2p_table[i].mv - mv) * dp + dv / 2) / dv;
        }
    }
    return 0;
}
static int mt6351_auxadc_init(struct mt6351_bat *bat)
{
    int ret;

    if (bat->init_done)
        return 0;

    /* Enable AUXADC clock always-on */
    ret = regmap_update_bits(bat->regmap, MT6351_AUXADC_CON0,
                             BIT(MT6351_AUXADC_CK_AON_SHIFT),
                             BIT(MT6351_AUXADC_CK_AON_SHIFT));
    if (ret)
        return ret;

    /* Disable clock power-down */
    ret = regmap_update_bits(bat->regmap, MT6351_TOP_CKPDN_CON1,
                             BIT(MT6351_RG_AUXADC_CK_PDN_SHIFT), 0);
    if (ret)
        return ret;

    ret = regmap_update_bits(bat->regmap, MT6351_TOP_CKPDN_CON0,
                             BIT(MT6351_RG_AUXADC_SMPS_CK_PDN_SHIFT), 0);
    if (ret)
        return ret;

    /* Release HW clock gating */
    ret = regmap_write(bat->regmap, MT6351_TOP_CKHWEN_CON0_CLR, 0x818);
    if (ret)
        return ret;

    bat->init_done = true;
    dev_info(bat->dev, "AUXADC initialized\n");
    return 0;
}

static int mt6351_read_vbat(struct mt6351_bat *bat, int *mv)
{
    unsigned int val;
    int ret, loop = 0, raw;

    /* Request conversion on channel 0 */
    ret = regmap_write(bat->regmap, MT6351_AUXADC_RQST0_SET,
                       MT6351_AUXADC_RQST_CH0);
    if (ret)
        return ret;

    /* Poll for ready */
    do {
        ret = regmap_read(bat->regmap, MT6351_AUXADC_ADC23, &val);
        if (ret)
            return ret;

        if (val & BIT(MT6351_AUXADC_ADC_RDY_SHIFT))
            break;

        if (++loop > 200) {
            dev_err(bat->dev, "AUXADC timeout\n");
            return -ETIMEDOUT;
        }
        udelay(50);
    } while (1);

    raw = val & MT6351_AUXADC_ADC_OUT_MASK;

    /* (raw * 3 * 1800) / 32768 — без /2: BY_AP-канал 15-бит, RDY уже отрезан маской */
    *mv = (raw * R_VAL_TEMP_CH0 * VOLTAGE_FULL_RANGE) / 32768;

    dev_info(bat->dev, "AUXADC raw=%d voltage=%d mV\n", raw, *mv);
    return 0;
}

static struct power_supply *mt6351_get_charger(struct mt6351_bat *bat)
{
    if (bat->charger)
        return bat->charger;

    /* имя зависит от версии ядра: с суффиксом -0 или без */
    bat->charger = power_supply_get_by_name("bq25890-charger-0");
    if (!bat->charger)
        bat->charger = power_supply_get_by_name("bq25890-charger");

    return bat->charger;
}

static int mt6351_bat_status(struct mt6351_bat *bat)
{
    struct power_supply *chg = mt6351_get_charger(bat);
    union power_supply_propval v;
    int ret;

    if (!chg)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    /* Сначала пробуем STATUS — bq25890 сам знает про Charging/Full */
    ret = power_supply_get_property(chg, POWER_SUPPLY_PROP_STATUS, &v);
    if (!ret && v.intval != POWER_SUPPLY_STATUS_UNKNOWN)
        return v.intval;

    /* Фолбэк: смотрим только наличие VBUS */
    ret = power_supply_get_property(chg, POWER_SUPPLY_PROP_ONLINE, &v);
    if (ret)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    return v.intval ? POWER_SUPPLY_STATUS_CHARGING :
    POWER_SUPPLY_STATUS_DISCHARGING;
}
/* Vendor-латч FGADC: READ-команда в старший байт CON0 (СОХРАНЯЯ младший!),
 * ждать бит10, читать CON11, SW-clear, ждать сброса, обнулить старший байт.
 * Возвращает ток в мкА: + = заряд, − = разряд. */
static int mt6351_fg_read_current(struct mt6351_bat *bat, int *ua)
{
    unsigned int con0 = 0, val = 0;
    unsigned int low;
    unsigned int mag;
    bool charging;
    int i, ret, r;
    s64 tmp;

    ret = regmap_read(bat->regmap, MT6351_FGADC_CON0, &con0);
    if (ret)
        return ret;
    low = con0 & 0x00ff;

    ret = regmap_write(bat->regmap, MT6351_FGADC_CON0, low | 0x0200);
    if (ret)
        return ret;

    for (i = 0; i < 20; i++) {
        regmap_read(bat->regmap, MT6351_FGADC_CON0, &con0);
        if (con0 & MT6351_FG_LATCHDATA_ST)
            break;
        usleep_range(500, 1000);
    }
    if (!(con0 & MT6351_FG_LATCHDATA_ST))
        return -ETIMEDOUT;

    ret = regmap_read(bat->regmap, MT6351_FGADC_CON11, &val);
    if (ret)
        goto restore;

    /* SW clear */
    regmap_write(bat->regmap, MT6351_FGADC_CON0, low | 0x0800);
    for (i = 0; i < 20; i++) {
        regmap_read(bat->regmap, MT6351_FGADC_CON0, &con0);
        if (!(con0 & MT6351_FG_LATCHDATA_ST))
            break;
        usleep_range(500, 1000);
    }
restore:
    regmap_write(bat->regmap, MT6351_FGADC_CON0, low);
    if (ret)
        return ret;

    /* uvalue16 > 0x7FFF -> разряд (дополнение до 2) */
    if ((val & 0xffff) > 32767) {
        mag = 65536 - (val & 0xffff);
        charging = false;
    } else {
        mag = val & 0xffff;
        charging = true;
    }

    /* uA = counts * 158.122 * (20 / R_FG) */
    tmp = (s64)mag * MT6351_FG_UNIT_UA_x1000 * 20;
    r = fg_r_mohm;
    if (r <= 0)
        r = 10;             /* защита от кривого module param */
    do_div(tmp, (u64)r * 1000);
    if (!charging)
        tmp = -tmp;

    *ua = (int)tmp;
    return 0;
}
static void mt6351_bat_poll(struct work_struct *work)
{
    struct mt6351_bat *bat = container_of(to_delayed_work(work),
                                          struct mt6351_bat, poll_work);
    ktime_t now = ktime_get();
    int status, ua = 0, ret, pct;
    s64 dt_ms;

    mutex_lock(&bat->lock);
    status = mt6351_bat_status(bat);

    /* ток FGADC; при сбое держим предыдущее значение */
    ret = mt6351_fg_read_current(bat, &ua);
    if (!ret)
        bat->fg_ua = ua;

    /* кулон-интегрирование: uA*ms / 3.6e6 = uAh */
    dt_ms = ktime_ms_delta(now, bat->last_ts);
    bat->last_ts = now;
    if (dt_ms > 0 && dt_ms < 60000) {
        /* ВАЖНО: только div64_s64. do_div() — беззнаковый: на разряде дельта
         * отрицательная, и do_div превращает её в огромное положительное
         * число -> charge_uah мгновенно упирается в C_full (100% при
         * отключении зарядки). Симптом 19.11: CAPACITY 38% -> 100% сразу
         * после выдёргивания кабеля и держится всё время разряда. */
        s64 delta_uah = div64_s64((s64)bat->fg_ua * dt_ms, 3600000);

        bat->charge_uah += delta_uah;
    }
    if (bat->charge_uah < 0)
        bat->charge_uah = 0;
    if (bat->charge_uah > MT6351_BAT_CHARGE_FULL)
        bat->charge_uah = MT6351_BAT_CHARGE_FULL;

    /* Якоря SOC:
     * 1) терминация (Full) -> 100%: только после 3 подряд опросов со статусом FULL
     *    и малым током (ложный FULL от bq на CV-переходах иначе прыгает в 100%);
     * 2) OCV в покое: |I| < 5 мА и не заряжаемся -> напряжение = OCV,
     *    таблица задаёт заряд. Поэтому при подключении зарядки % НЕ прыгает:
     *    живое (завышенное) напряжение никогда не попадает в таблицу. */
    if (status == POWER_SUPPLY_STATUS_FULL &&
        abs(bat->fg_ua) < MT6351_FG_FULL_ANCHOR_UA) {
        bat->full_count++;
        if (bat->full_count >= 3) {
            bat->charge_uah = MT6351_BAT_CHARGE_FULL;
            bat->full_count = 0;
        }
        bat->rest_count = 0;
    } else if (status == POWER_SUPPLY_STATUS_FULL) {
        /* FULL при большом токе — не верим (CV-переход), якорь не ставим */
        bat->full_count = 0;
        bat->rest_count = 0;
    } else if (abs(bat->fg_ua) < MT6351_FG_OCV_REST_UA &&
           status != POWER_SUPPLY_STATUS_CHARGING) {
        int mv;

        bat->rest_count++;
        if (bat->rest_count >= MT6351_FG_OCV_REST_POLLS &&
            mt6351_read_vbat(bat, &mv) == 0) {
            pct = mt6351_v2p(mv);
            bat->charge_uah = (s64)pct * MT6351_BAT_CHARGE_FULL / 100;
            bat->cached_mv = mv;
        }
    } else {
        bat->rest_count = 0;
    }

    pct = (int)div64_s64(bat->charge_uah * 100, MT6351_BAT_CHARGE_FULL);
    bat->cached_pct = pct;
    mutex_unlock(&bat->lock);

    if (status != bat->last_status) {
        bat->last_status = status;
        power_supply_changed(bat->psy);
    } else if (abs(pct - bat->last_pct) >= 1) {
        power_supply_changed(bat->psy);
    }
    bat->last_pct = pct;

    schedule_delayed_work(&bat->poll_work, MT6351_POLL_INTERVAL);
}
static int mt6351_bat_get_prop(struct power_supply *psy,
                               enum power_supply_property psp,
                               union power_supply_propval *val)
{
    struct mt6351_bat *bat = power_supply_get_drvdata(psy);
    int ret = 0, mv;

    mutex_lock(&bat->lock);

    switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
            val->intval = mt6351_bat_status(bat);
            break;

        case POWER_SUPPLY_PROP_PRESENT:
            val->intval = 1;
            break;

        case POWER_SUPPLY_PROP_TECHNOLOGY:
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            ret = mt6351_auxadc_init(bat);
            if (ret)
                break;
        ret = mt6351_read_vbat(bat, &mv);
        if (ret)
            break;
        bat->cached_mv = mv;
        val->intval = mv * 1000;
        break;

        case POWER_SUPPLY_PROP_CAPACITY:
            /* SOC из кулон-счёта — гладкий, не прыгает при подключении заряда.
             * Напряжение участвует только через OCV-якорь в покое (poll). */
            val->intval = bat->cached_pct;
            break;
        case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        case POWER_SUPPLY_PROP_CHARGE_FULL:
            val->intval = MT6351_BAT_CHARGE_FULL;
            break;

        case POWER_SUPPLY_PROP_CHARGE_NOW:
            val->intval = (int)bat->charge_uah;
            break;

        case POWER_SUPPLY_PROP_CURRENT_NOW:
            /* мкА, + = заряд (знак FGADC vendor-а совпадает с конвенцией) */
            val->intval = bat->fg_ua;
            break;
        default:
            ret = -EINVAL;
    }

    mutex_unlock(&bat->lock);
    return ret;
}

static enum power_supply_property mt6351_bat_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_NOW,
};

static const struct power_supply_desc mt6351_bat_desc = {
    .name = "mt6351-battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = mt6351_bat_props,
    .num_properties = ARRAY_SIZE(mt6351_bat_props),
    .get_property = mt6351_bat_get_prop,
};

/* Заряд в мкА·ч — сохраняется/восстанавливается userspace-демоном
 * (SOC переживает перезагрузку). Запись клампится в [0, ёмкость]. */
static ssize_t charge_uah_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct mt6351_bat *bat = dev_get_drvdata(dev);
    s64 v;

    if (!bat)
        return -ENODEV;
    mutex_lock(&bat->lock);
    v = bat->charge_uah;
    mutex_unlock(&bat->lock);
    return sprintf(buf, "%lld\n", v);
}

static ssize_t charge_uah_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct mt6351_bat *bat = dev_get_drvdata(dev);
    s64 v;
    int ret;

    if (!bat)
        return -ENODEV;
    ret = kstrtoll(buf, 10, &v);
    if (ret)
        return ret;
    if (v < 0)
        v = 0;
    if (v > MT6351_BAT_CHARGE_FULL)
        v = MT6351_BAT_CHARGE_FULL;
    mutex_lock(&bat->lock);
    bat->charge_uah = v;
    bat->cached_pct = (int)div64_s64(v * 100, MT6351_BAT_CHARGE_FULL);
    mutex_unlock(&bat->lock);
    return count;
}
static DEVICE_ATTR_RW(charge_uah);

static int mt6351_bat_probe(struct platform_device *pdev)
{
    struct mt6351_bat *bat;
    struct power_supply_config cfg = {};
    int ret;

    bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
    if (!bat)
        return -ENOMEM;

    bat->dev = &pdev->dev;
    mutex_init(&bat->lock);

    /* regmap lives on pwrap (grandparent) */
    bat->regmap = dev_get_regmap(pdev->dev.parent->parent, NULL);
    if (!bat->regmap) {
        dev_err(&pdev->dev, "no regmap\n");
        return -ENODEV;
    }

    platform_set_drvdata(pdev, bat);

    ret = mt6351_auxadc_init(bat);
    if (ret)
        return ret;

    cfg.drv_data = bat;
    cfg.of_node = pdev->dev.of_node;

    bat->psy = devm_power_supply_register(&pdev->dev,
                                          &mt6351_bat_desc, &cfg);
    if (IS_ERR(bat->psy))
        return PTR_ERR(bat->psy);

    bat->last_status = mt6351_bat_status(bat);

    /* стартовая оценка SOC: раз кулон-история не сохраняется между
     * загрузками, берём OCV-таблицу (на реальной OCV в покое точна,
     * под нагрузкой чуть занижена — OCV-якорь скорректирует). */
    if (mt6351_read_vbat(bat, &bat->cached_mv) == 0) {
        bat->charge_uah = (s64)mt6351_v2p(bat->cached_mv) *
                           MT6351_BAT_CHARGE_FULL / 100;
    } else {
        bat->charge_uah = MT6351_BAT_CHARGE_FULL / 2;
    }
    bat->last_pct = bat->cached_pct =
        (int)div64_s64(bat->charge_uah * 100, MT6351_BAT_CHARGE_FULL);
    bat->fg_ua = 0;
    bat->rest_count = 0;
    bat->last_ts = ktime_get();

    INIT_DELAYED_WORK(&bat->poll_work, mt6351_bat_poll);


    schedule_delayed_work(&bat->poll_work, MT6351_POLL_INTERVAL);

    ret = device_create_file(&pdev->dev, &dev_attr_charge_uah);
    if (ret)
        dev_warn(&pdev->dev, "charge_uah attr failed: %d\n", ret);

    dev_info(&pdev->dev, "MT6351 battery (AUXADC) probed\n");
    return 0;
}

static int mt6351_bat_remove(struct platform_device *pdev)
{
    struct mt6351_bat *bat = platform_get_drvdata(pdev);

    cancel_delayed_work_sync(&bat->poll_work);
    device_remove_file(&pdev->dev, &dev_attr_charge_uah);

    if (bat->charger)
        power_supply_put(bat->charger);

    return 0;
}

static const struct of_device_id mt6351_bat_match[] = {
    { .compatible = "mediatek,mt6351-battery" },
    { }
};
MODULE_DEVICE_TABLE(of, mt6351_bat_match);

static struct platform_driver mt6351_bat_driver = {
    .driver = {
        .name = "mt6351-battery",
        .of_match_table = mt6351_bat_match,
    },
    .probe = mt6351_bat_probe,
    .remove = mt6351_bat_remove,
};
module_platform_driver(mt6351_bat_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6351 Battery via AUXADC");
