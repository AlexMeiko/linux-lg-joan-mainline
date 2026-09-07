// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PMI8998 fuel gauge driver.
 *
 * The fuel-gauge algorithm runs in hardware. This driver only exposes its
 * BATT_SOC and BATT_INFO shadow registers through the power-supply class.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <soc/qcom/qcom-spmi-pmic.h>

#define PMI8998_FG_PERPH_SUBTYPE_REG	0x05
#define PMI8998_FG_SOC_SUBTYPE		0x10
#define PMI8998_FG_INFO_SUBTYPE		0x11

/* Monotonic SOC is valid only when both shadow bytes match. */
#define PMI8998_FG_SOC_CAPACITY		0x09

#define PMI8998_FG_INFO_TEMP_LSB		0x50
#define PMI8998_FG_INFO_VOLT_LSB		0xa0
#define PMI8998_FG_INFO_CURRENT_LSB	0xa2

#define PMI8998_FG_INFO_OFFSET		0x100
#define PMI8998_FG_SOC_SHADOW_RETRIES	5

struct pmi8998_fg {
	struct device *dev;
	struct regmap *regmap;
	unsigned int soc_base;
	unsigned int info_base;
	bool v1;
};

static int pmi8998_fg_read_block(struct pmi8998_fg *fg, unsigned int addr,
				 u8 *buf, size_t len)
{
	return regmap_bulk_read(fg->regmap, addr, buf, len);
}

static int pmi8998_fg_check_revision(struct pmi8998_fg *fg)
{
	const struct qcom_spmi_pmic *pmic;

	pmic = qcom_pmic_get(fg->dev);
	if (IS_ERR(pmic))
		return dev_err_probe(fg->dev, PTR_ERR(pmic),
				     "failed to read PMIC revision\n");

	if (pmic->subtype != PMI8998_SUBTYPE)
		return dev_err_probe(fg->dev, -ENODEV,
				     "unsupported PMIC subtype 0x%x\n",
				     pmic->subtype);

	/* Downstream uses the v1 workaround for every major revision below 2. */
	if (pmic->major < 2) {
		fg->v1 = true;
	} else if (pmic->major != 2) {
		return dev_err_probe(fg->dev, -ENODEV,
				     "unsupported PMI8998 revision v%u.%u\n",
				     pmic->major, pmic->minor);
	}

	dev_info(fg->dev, "PMI8998 v%u.%u shadow register layout\n",
		 pmic->major, pmic->minor);

	return 0;
}

static int pmi8998_fg_check_subtype(struct pmi8998_fg *fg, unsigned int base,
				    unsigned int expected, const char *name)
{
	unsigned int subtype;
	int ret;

	ret = regmap_read(fg->regmap, base + PMI8998_FG_PERPH_SUBTYPE_REG,
			  &subtype);
	if (ret)
		return ret;

	if (subtype != expected) {
		dev_err(fg->dev, "%s subtype 0x%x does not match 0x%x\n",
			name, subtype, expected);
		return -ENODEV;
	}

	return 0;
}

static u16 pmi8998_fg_decode_word(struct pmi8998_fg *fg, const u8 *buf)
{
	return fg->v1 ? get_unaligned_be16(buf) : get_unaligned_le16(buf);
}

static int pmi8998_fg_read_soc_raw(struct pmi8998_fg *fg, u8 *raw)
{
	u8 soc[2];
	int ret, i;

	for (i = 0; i < PMI8998_FG_SOC_SHADOW_RETRIES; i++) {
		ret = pmi8998_fg_read_block(fg,
					    fg->soc_base + PMI8998_FG_SOC_CAPACITY,
					    soc, sizeof(soc));
		if (ret)
			return ret;

		if (soc[0] == soc[1]) {
			*raw = soc[0];
			return 0;
		}
	}

	dev_dbg(fg->dev, "SOC shadow values do not match\n");
	return -EAGAIN;
}

static int pmi8998_fg_get_capacity(struct pmi8998_fg *fg, int *val)
{
	u8 raw;
	int ret;

	ret = pmi8998_fg_read_soc_raw(fg, &raw);
	if (ret)
		return ret;

	if (raw == 0) {
		*val = 0;
		return 0;
	}

	if (raw == 255) {
		*val = 100;
		return 0;
	}

	*val = DIV_ROUND_CLOSEST((raw - 1) * 98, 253) + 1;
	return 0;
}

static int pmi8998_fg_get_voltage(struct pmi8998_fg *fg, int *val)
{
	u8 buf[2];
	u16 raw;
	int ret;

	ret = pmi8998_fg_read_block(fg, fg->info_base + PMI8998_FG_INFO_VOLT_LSB,
				    buf, sizeof(buf));
	if (ret)
		return ret;

	raw = pmi8998_fg_decode_word(fg, buf);
	*val = div_u64((u64)raw * 122070ULL, 1000ULL);
	return 0;
}

static int pmi8998_fg_get_current(struct pmi8998_fg *fg, int *val)
{
	u8 buf[2];
	s64 microamp;
	s32 raw;
	int ret;

	ret = pmi8998_fg_read_block(fg,
				    fg->info_base + PMI8998_FG_INFO_CURRENT_LSB,
				    buf, sizeof(buf));
	if (ret)
		return ret;

	raw = sign_extend32(pmi8998_fg_decode_word(fg, buf), 15);
	microamp = div_s64((s64)raw * 488281LL, 1000LL);
	*val = microamp;

	return 0;
}

static int pmi8998_fg_get_temp(struct pmi8998_fg *fg, int *val)
{
	u8 buf[2];
	unsigned int raw;
	int ret;

	ret = pmi8998_fg_read_block(fg, fg->info_base + PMI8998_FG_INFO_TEMP_LSB,
				    buf, sizeof(buf));
	if (ret)
		return ret;

	raw = ((buf[1] & GENMASK(2, 0)) << 8) | buf[0];
	*val = DIV_ROUND_CLOSEST(raw * 10, 4) - 2730;
	return 0;
}

static int pmi8998_fg_get_status(struct power_supply *psy, int *val)
{
	union power_supply_propval status;
	int ret;

	ret = power_supply_get_property_from_supplier(psy,
						      POWER_SUPPLY_PROP_STATUS,
						      &status);
	if (ret == -ENODEV) {
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}
	if (ret)
		return ret;

	*val = status.intval;
	return 0;
}

static int pmi8998_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pmi8998_fg *fg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return pmi8998_fg_get_status(psy, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return pmi8998_fg_get_capacity(fg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return pmi8998_fg_get_voltage(fg, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return pmi8998_fg_get_current(fg, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return pmi8998_fg_get_temp(fg, &val->intval);
	default:
		return -EINVAL;
	}
}

static void pmi8998_fg_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static const enum power_supply_property pmi8998_fg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc pmi8998_fg_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pmi8998_fg_properties,
	.num_properties = ARRAY_SIZE(pmi8998_fg_properties),
	.get_property = pmi8998_fg_get_property,
	.external_power_changed = pmi8998_fg_external_power_changed,
};

static int pmi8998_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct power_supply *psy;
	struct pmi8998_fg *fg;
	u32 base;
	int ret;

	fg = devm_kzalloc(&pdev->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = &pdev->dev;
	fg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(fg->dev, -ENODEV,
				     "failed to locate the regmap\n");

	ret = pmi8998_fg_check_revision(fg);
	if (ret)
		return ret;

	ret = device_property_read_u32(fg->dev, "reg", &base);
	if (ret)
		return dev_err_probe(fg->dev, ret, "failed to read reg base\n");

	fg->soc_base = base;
	fg->info_base = base + PMI8998_FG_INFO_OFFSET;

	ret = pmi8998_fg_check_subtype(fg, fg->soc_base,
				       PMI8998_FG_SOC_SUBTYPE, "batt-soc");
	if (ret)
		return ret;

	ret = pmi8998_fg_check_subtype(fg, fg->info_base,
				       PMI8998_FG_INFO_SUBTYPE, "batt-info");
	if (ret)
		return ret;

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	psy = devm_power_supply_register(&pdev->dev, &pmi8998_fg_desc,
					 &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(psy),
				     "failed to register battery power supply\n");

	platform_set_drvdata(pdev, fg);

	return 0;
}

static const struct of_device_id pmi8998_fg_of_match[] = {
	{ .compatible = "qcom,pmi8998-fg" },
	{ }
};
MODULE_DEVICE_TABLE(of, pmi8998_fg_of_match);

static struct platform_driver pmi8998_fg_driver = {
	.probe = pmi8998_fg_probe,
	.driver = {
		.name = "qcom-pmi8998-fg",
		.of_match_table = pmi8998_fg_of_match,
	},
};
module_platform_driver(pmi8998_fg_driver);

MODULE_AUTHOR("GuWolf <admin@guwolf.com>");
MODULE_DESCRIPTION("Qualcomm PMI8998 fuel gauge driver");
MODULE_LICENSE("GPL");
