// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* MT6755 I2C register offsets */
#define I2C_DATA_PORT        0x00
#define I2C_SLAVE_ADDR       0x04
#define I2C_INTR_MASK        0x08
#define I2C_INTR_STAT        0x0C
#define I2C_CONTROL          0x10
#define I2C_TRANSFER_LEN     0x14
#define I2C_TRANSAC_LEN      0x18
#define I2C_DELAY_LEN        0x1C
#define I2C_TIMING           0x20
#define I2C_START            0x24
#define I2C_EXT_CONF         0x28
#define I2C_FIFO_STAT        0x30
#define I2C_FIFO_THRESH      0x34
#define I2C_FIFO_ADDR_CLR    0x38
#define I2C_IO_CONFIG        0x40
#define I2C_HS               0x48
#define I2C_SOFTRESET        0x50
#define I2C_DCM_EN           0x54
#define I2C_DEBUGSTAT        0x64
#define I2C_TRANSFER_LEN_AUX 0x6C

/* Control register bits */
#define I2C_CONTROL_RS              (1 << 1)
#define I2C_CONTROL_DMA_EN          (1 << 2)
#define I2C_CONTROL_CLK_EXT_EN      (1 << 3)
#define I2C_CONTROL_DIR_CHANGE      (1 << 4)
#define I2C_CONTROL_ACKERR_DET_EN   (1 << 5)

/* Interrupt status bits */
#define I2C_TRANSAC_COMP  (1 << 0)
#define I2C_ACKERR        (1 << 1)
#define I2C_HS_NACKERR    (1 << 2)

#define I2C_FIFO_SIZE      8
#define I2C_TIMEOUT_MS     1000

struct mt6755_i2c {
    struct i2c_adapter adap;
    void __iomem *base;
    struct device *dev;
    u32 speed;
};

static inline void mt_i2c_writew(struct mt6755_i2c *i2c, u32 offset, u16 val)
{
    writel(val, i2c->base + offset);
}

static inline u16 mt_i2c_readw(struct mt6755_i2c *i2c, u32 offset)
{
    return readl(i2c->base + offset) & 0xFFFF;
}

static void mt_i2c_init_hw(struct mt6755_i2c *i2c)
{
    /* Soft reset */
    mt_i2c_writew(i2c, I2C_SOFTRESET, 0x0001);
    udelay(10);

    /* Disable DCM */
    mt_i2c_writew(i2c, I2C_DCM_EN, 0x0000);

    /* Clear interrupt status */
    mt_i2c_writew(i2c, I2C_INTR_STAT,
                  I2C_TRANSAC_COMP | I2C_ACKERR | I2C_HS_NACKERR);

    /* Clear FIFO address */
    mt_i2c_writew(i2c, I2C_FIFO_ADDR_CLR, 0x0001);
}

static int mt_i2c_wait_complete(struct mt6755_i2c *i2c)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(I2C_TIMEOUT_MS);
    u16 stat;

    while (time_before(jiffies, timeout)) {
        stat = mt_i2c_readw(i2c, I2C_INTR_STAT);
        if (stat & (I2C_TRANSAC_COMP | I2C_ACKERR | I2C_HS_NACKERR)) {
            mt_i2c_writew(i2c, I2C_INTR_STAT, stat);

            if (stat & (I2C_ACKERR | I2C_HS_NACKERR)) {
                dev_dbg(i2c->dev, "I2C NACK error, stat=0x%04x\n", stat);
                return -EREMOTEIO;
            }
            return 0;
        }
        udelay(10);
    }

    dev_err(i2c->dev, "I2C transfer timeout\n");
    return -ETIMEDOUT;
}

static int mt_i2c_xfer_msg(struct mt6755_i2c *i2c, struct i2c_msg *msg, bool use_rs)
{
    bool is_read = msg->flags & I2C_M_RD;
    u16 control, timing;
    int ret;

    /* Timing for ~100kHz (standard mode) */
    timing = ((13 - 1) << 8) | (13 - 1);
    mt_i2c_writew(i2c, I2C_TIMING, timing);

    /* Control register */
    control = I2C_CONTROL_CLK_EXT_EN | I2C_CONTROL_ACKERR_DET_EN;
    if (use_rs)
        control |= I2C_CONTROL_RS;
    mt_i2c_writew(i2c, I2C_CONTROL, control);

    /* Extended configuration */
    if (i2c->speed <= 100)
        mt_i2c_writew(i2c, I2C_EXT_CONF, 0x8001);
    else
        mt_i2c_writew(i2c, I2C_EXT_CONF, 0x1800);

    /* Delay length */
    mt_i2c_writew(i2c, I2C_DELAY_LEN, 0x0002);

    /* IO config (open-drain) */
    mt_i2c_writew(i2c, I2C_IO_CONFIG, 0x0003);

    /* Slave address (7-bit, hardware shifts it) */
    mt_i2c_writew(i2c, I2C_SLAVE_ADDR, (msg->addr << 1));

    /* Disable interrupts (polling mode) */
    mt_i2c_writew(i2c, I2C_INTR_MASK, 0x0000);

    /* Transfer length */
    mt_i2c_writew(i2c, I2C_TRANSFER_LEN, msg->len);

    /* Transaction count */
    mt_i2c_writew(i2c, I2C_TRANSAC_LEN, 1);

    /* Write data to FIFO if writing */
    if (!is_read) {
        int j;
        for (j = 0; j < msg->len && j < I2C_FIFO_SIZE; j++) {
            mt_i2c_writew(i2c, I2C_DATA_PORT, msg->buf[j]);
        }
    }

    /* Start transfer */
    mt_i2c_writew(i2c, I2C_START, 0x0001);

    /* Wait for completion */
    ret = mt_i2c_wait_complete(i2c);
    if (ret < 0) {
        mt_i2c_writew(i2c, I2C_SOFTRESET, 0x0001);
        return ret;
    }

    /* Read data from FIFO if reading */
    if (is_read) {
        int j;
        u16 fifo_stat = mt_i2c_readw(i2c, I2C_FIFO_STAT);
        int data_size = (fifo_stat >> 4) & 0x0F;

        for (j = 0; j < data_size && j < msg->len; j++) {
            msg->buf[j] = mt_i2c_readw(i2c, I2C_DATA_PORT) & 0xFF;
        }
    }

    return 0;
}

static int mt_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    struct mt6755_i2c *i2c = i2c_get_adapdata(adap);
    int i, ret;
    u16 control, timing;

    for (i = 0; i < num; i++) {
        struct i2c_msg *msg = &msgs[i];
        bool is_read = msg->flags & I2C_M_RD;
        bool is_last = (i == num - 1);

        /* Initialize hardware for each message */
        mt_i2c_init_hw(i2c);

        /* Set timing for 100kHz */
        timing = ((13 - 1) << 8) | (13 - 1);
        mt_i2c_writew(i2c, I2C_TIMING, timing);

        /* Set control register */
        control = I2C_CONTROL_CLK_EXT_EN | I2C_CONTROL_ACKERR_DET_EN;
        if (!is_last)
            control |= I2C_CONTROL_RS;
        mt_i2c_writew(i2c, I2C_CONTROL, control);

        /* Set extended configuration */
        if (i2c->speed <= 100)
            mt_i2c_writew(i2c, I2C_EXT_CONF, 0x8001);
        else
            mt_i2c_writew(i2c, I2C_EXT_CONF, 0x1800);

        /* Set delay length */
        mt_i2c_writew(i2c, I2C_DELAY_LEN, 0x0002);

        /* Set IO config (open-drain) */
        mt_i2c_writew(i2c, I2C_IO_CONFIG, 0x0003);

        /* КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Set slave address WITH R/W bit! */
        if (is_read)
            mt_i2c_writew(i2c, I2C_SLAVE_ADDR, (msg->addr << 1) | 0x1);
        else
            mt_i2c_writew(i2c, I2C_SLAVE_ADDR, (msg->addr << 1) & ~0x1);

        /* Disable interrupts (polling mode) */
        mt_i2c_writew(i2c, I2C_INTR_MASK, 0x0000);

        /* Set transfer length */
        mt_i2c_writew(i2c, I2C_TRANSFER_LEN, msg->len);

        /* Set transaction count */
        mt_i2c_writew(i2c, I2C_TRANSAC_LEN, 1);

        /* Write data to FIFO if writing */
        if (!is_read) {
            int j;
            for (j = 0; j < msg->len && j < I2C_FIFO_SIZE; j++) {
                mt_i2c_writew(i2c, I2C_DATA_PORT, msg->buf[j]);
            }
        }

        /* Start transfer */
        mt_i2c_writew(i2c, I2C_START, 0x0001);

        /* Wait for completion */
        ret = mt_i2c_wait_complete(i2c);
        if (ret < 0) {
            mt_i2c_writew(i2c, I2C_SOFTRESET, 0x0001);
            return ret;
        }

        /* Read data from FIFO if reading */
        if (is_read) {
            int j;
            u16 fifo_stat = mt_i2c_readw(i2c, I2C_FIFO_STAT);
            int data_size = (fifo_stat >> 4) & 0x0F;

            for (j = 0; j < data_size && j < msg->len; j++) {
                msg->buf[j] = mt_i2c_readw(i2c, I2C_DATA_PORT) & 0xFF;
            }
        }
    }

    return num;
}

static u32 mt_i2c_functionality(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm mt_i2c_algo = {
    .master_xfer = mt_i2c_xfer,
    .functionality = mt_i2c_functionality,
};

static int mt_i2c_probe(struct platform_device *pdev)
{
    struct mt6755_i2c *i2c;
    struct resource *res;
    u32 speed = 100000;
    int ret;

    i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
    if (!i2c)
        return -ENOMEM;

    i2c->dev = &pdev->dev;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    i2c->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(i2c->base))
        return PTR_ERR(i2c->base);

    of_property_read_u32(pdev->dev.of_node, "clock-frequency", &speed);
    i2c->speed = speed / 1000;

    i2c->adap.owner = THIS_MODULE;
    i2c->adap.algo = &mt_i2c_algo;
    i2c->adap.dev.parent = &pdev->dev;
    i2c->adap.dev.of_node = pdev->dev.of_node;
    snprintf(i2c->adap.name, sizeof(i2c->adap.name), "mt6755-i2c");

    /* Динамическая нумерация — избегаем -EBUSY */
    i2c->adap.nr = -1;

    i2c_set_adapdata(&i2c->adap, i2c);
    platform_set_drvdata(pdev, i2c);

    mt_i2c_init_hw(i2c);

    ret = i2c_add_adapter(&i2c->adap);
    if (ret) {
        dev_err(&pdev->dev, "failed to add i2c adapter (ret=%d)\n", ret);
        return ret;
    }

    dev_info(&pdev->dev, "MT6755 I2C adapter registered (bus %d)\n", i2c->adap.nr);
    return 0;
}

static int mt_i2c_remove(struct platform_device *pdev)
{
    struct mt6755_i2c *i2c = platform_get_drvdata(pdev);

    i2c_del_adapter(&i2c->adap);
    return 0;
}

static const struct of_device_id mt_i2c_of_match[] = {
    { .compatible = "mediatek,mt6755-i2c" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt_i2c_of_match);

static struct platform_driver mt_i2c_driver = {
    .probe = mt_i2c_probe,
    .remove = mt_i2c_remove,
    .driver = {
        .name = "mt6755-i2c",
        .of_match_table = mt_i2c_of_match,
    },
};

module_platform_driver(mt_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mainline Porting");
MODULE_DESCRIPTION("MT6755 I2C minimal driver");
