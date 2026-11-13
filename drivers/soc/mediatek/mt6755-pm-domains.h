/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_MEDIATEK_MT6755_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT6755_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mt6755-power.h>
#define SPM_PWR_STATUS               0x180
#define SPM_PWR_STATUS_2ND           0x184

/* Values from the stock 3.18 kernel: spm_mtcmos_ctrl_mfg_async/mfg2 */
static const struct scpsys_domain_data scpsys_domain_data_mt6755[] = {
    [MT6755_POWER_DOMAIN_MFG_ASYNC] = {
        .name = "mfg_async",
        .sta_mask = BIT(23),
        .ctl_offs = 0x334,
        .pwr_sta_offs = SPM_PWR_STATUS,
        .pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
    },
    [MT6755_POWER_DOMAIN_MFG] = {
        .name = "mfg",
        .sta_mask = BIT(4),
        .ctl_offs = 0x338,
        .pwr_sta_offs = SPM_PWR_STATUS,
        .pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
        .sram_pdn_bits = BIT(8),
        .sram_pdn_ack_bits = BIT(16),
        .bp_infracfg = {
            BUS_PROT_UPDATE_TOPAXI(BIT(21)),
        },
    },
};

static const struct scpsys_soc_data mt6755_scpsys_data = {
    .domains_data = scpsys_domain_data_mt6755,
    .num_domains = ARRAY_SIZE(scpsys_domain_data_mt6755),
    .spm_poweron_cfg = 0x0b160001,
};

#endif
