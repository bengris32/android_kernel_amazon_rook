/*
 *
 * Filename: hx8379c_wvga_dsi_vdo_rook.c
 * Description:  Himax 8379c display driver IC
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 *
 * Deepak Bhople (bhopled@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/string.h>
#include <linux/gpio.h>
#include <mt-plat/mt_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#include "ddp_hal.h"

#include "lcm_drv.h"

/* ------------------------------------------------------------------------- */
/* Local Constants */
/* ------------------------------------------------------------------------- */
#define FRAME_WIDTH  (480)  /* HX8379C for BOE 3.1" resolution 480x800 */
#define FRAME_HEIGHT (480)

#define LCM_ID       (0x8379) /* HX8379C for BOE 3.1" */

#define REGFLAG_DELAY								0xFE
#define REGFLAG_END_OF_TABLE							0xFF
/* ------------------------------------------------------------------------- */
/* Local Variables */
/* ------------------------------------------------------------------------- */
static LCM_UTIL_FUNCS lcm_util = { 0 };


#define UDELAY(n)		(lcm_util.udelay(n))
#define MDELAY(n)		(lcm_util.mdelay(n))

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

/* ------------------------------------------------------------------------- */
/* Local Functions */
/* ------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				         lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg					 lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V3(para_tbl, size, force_update)    lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)

extern void DSI_clk_HS_mode(DISP_MODULE_ENUM module, void *cmdq, bool enter);
/* static void lcm_panel_init(void); */
static int vid = -1;

/* ------------------------------------------------------------------------ */
/* Local structure enum constant */
/* ------------------------------------------------------------------------ */
enum PANEL_VENDOR {
	INX = 0,
	TIANMA,
	BOE,
	INXE
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/* ------------------------------------------------------------------------- */
/* static structure Implementations */
/* ------------------------------------------------------------------------- */
static struct pinctrl *lcmctrl;
static struct pinctrl_state *lcd_1p8_pwr_high;
static struct pinctrl_state *lcd_1p8_pwr_low;
static struct pinctrl_state *lcd_3p3_pwr_high;
static struct pinctrl_state *lcd_3p3_pwr_low;
static struct pinctrl_state *lcd_rst_high;
static struct pinctrl_state *lcd_rst_low;
static struct regulator *lcm_vcn_ldo;
static struct regulator *lcm_vmch_ldo;

#ifdef CONFIG_PANEL_BRIGHTNESS_SCALING
#define DUTY_CYCLE_RESOLUTION 10

int panel_brightness_scale[] = {
	[INX] = 943,
	[TIANMA] = 923,
	[BOE] = 963,
	[INXE] = 943
};

int lcm_get_panel_brightness_scale(void) {
	if (vid != -1)
		return panel_brightness_scale[vid];
	else
		return (1 << DUTY_CYCLE_RESOLUTION) - 1;
}

#ifdef CONFIG_PANEL_BRIGHTNESS_SCALING_DEBUG
static ssize_t brightness_scale_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return sprintf(buf, "%d\n", lcm_get_panel_brightness_scale());
}

static ssize_t brightness_scale_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len) {
	unsigned int new_scale = (1 << DUTY_CYCLE_RESOLUTION) - 1;
	if (kstrtouint(buf, 10, &new_scale)) {
		return -EINVAL;
	}

	pr_err("Setting brightness scaling to %d\n", new_scale);
	if (vid != -1) {
		panel_brightness_scale[vid] = new_scale;
	}
	return len;
}

DEVICE_ATTR_RW(brightness_scale);

int create_brightness_scale_debug_file(struct device *dev) {
	return device_create_file(dev, &dev_attr_brightness_scale);
}
#endif
#endif

/* ------------------------------------------------------------------------- */
/* lcm_get_vio_supply function Implementations */
/* ------------------------------------------------------------------------- */
static int lcm_get_vio_supply(struct device *dev)
{
	int ret;

	pr_info("[HX8379c] %s\n", __func__);
	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vcn_ldo = devm_regulator_get(dev, "lcm-vcn33");
	if (IS_ERR(lcm_vcn_ldo)) {
		ret = PTR_ERR(lcm_vcn_ldo);
		dev_err(dev, "failed to get lcm-vcn LDO, %d\n", ret);
		return ret;
	}

	lcm_vmch_ldo = devm_regulator_get(dev, "lcm-vmch");
	if (IS_ERR(lcm_vmch_ldo)) {
		ret = PTR_ERR(lcm_vmch_ldo);
		dev_err(dev, "failed to get lcm-vmch LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vcn_ldo);
	ret = regulator_enable(lcm_vmch_ldo);

	return ret;
}


/* ------------------------------------------------------------------------ */
/* lcm_get_gpio function Implementations */
/* ------------------------------------------------------------------------ */
static int lcm_get_gpio(struct device *dev)
{
	int ret = 0;

	pr_info("[HX8379c] %s\n", __func__);
	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	/* lcm power pin lookup */
	lcd_1p8_pwr_high = pinctrl_lookup_state(lcmctrl, "lcm_1p8_pwr_high");
	if (IS_ERR(lcd_1p8_pwr_high)) {
		ret = PTR_ERR(lcd_1p8_pwr_high);
		pr_info("%s : pinctrl err, lcd_1p8_pwr_high\n", __func__);
	}
	lcd_1p8_pwr_low = pinctrl_lookup_state(lcmctrl, "lcm_1p8_pwr_low");
	if (IS_ERR(lcd_1p8_pwr_low)) {
		ret = PTR_ERR(lcd_1p8_pwr_low);
		pr_info("%s : pinctrl err, lcd_1p8_pwr_low\n", __func__);
	}
	lcd_rst_high = pinctrl_lookup_state(lcmctrl, "lcm_rst_high");
	if (IS_ERR(lcd_rst_high)) {
		ret = PTR_ERR(lcd_rst_high);
		pr_info("%s : pinctrl err, lcd_rst_high\n", __func__);
	}
	lcd_rst_low = pinctrl_lookup_state(lcmctrl, "lcm_rst_low");
	if (IS_ERR(lcd_rst_low)) {
		ret = PTR_ERR(lcd_rst_low);
		pr_info("%s : pinctrl err, lcd_rst_low\n", __func__);
	}

	lcd_3p3_pwr_high = pinctrl_lookup_state(lcmctrl, "lcm_3p3_pwr_high");
	if (IS_ERR(lcd_3p3_pwr_high)) {
		ret = PTR_ERR(lcd_3p3_pwr_high);
		pr_info("%s : pinctrl err, lcd_3p3_pwr_high\n", __func__);
	}

	lcd_3p3_pwr_low = pinctrl_lookup_state(lcmctrl, "lcm_3p3_pwr_low");
	if (IS_ERR(lcd_3p3_pwr_low)) {
		ret = PTR_ERR(lcd_3p3_pwr_low);
		pr_info("%s : pinctrl err, lcd_3p3_pwr_low\n", __func__);
	}
	return ret;
}
/* ------------------------------------------------------------------------- */
/* lcm_set_pwr function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_set_pwr(int val)
{
#if 0
	if (val == 0) {
		pinctrl_select_state(lcmctrl, lcd_1p8_pwr_low);
	} else {
		pinctrl_select_state(lcmctrl, lcd_1p8_pwr_high);
	}
#endif
}

/* ------------------------------------------------------------------------- */
/* lcm_set_rst function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_set_rst(int val)
{
#if 0
	if (val == 0) {
		pinctrl_select_state(lcmctrl, lcd_rst_low);
	} else {
		pinctrl_select_state(lcmctrl, lcd_rst_high);
	}
#endif
}


/* ------------------------------------------------------------------------- */
/* lcm_set_pwr_n function  Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_set_pwr_n(int val)
{
#if 0
	if (val == 0 && (!IS_ERR(lcd_3p3_pwr_low)))  {
		pinctrl_select_state(lcmctrl, lcd_3p3_pwr_low);
	} else if (val == 1 && (!IS_ERR(lcd_3p3_pwr_high))) {
		pinctrl_select_state(lcmctrl, lcd_3p3_pwr_high);
	}
#endif
}
/* ------------------------------------------------------------------------- */
/* lcm_read_panel_id function Implementations */
/* ------------------------------------------------------------------------- */
static int lcm_read_panel_id(struct device *dev)
{
	unsigned int gpio_id0, gpio_id1;
	int val = -1;
	int ret = -1;

	pr_info("[HX8379c] %s\n", __func__);

	/* Read GPIO from device tree file*/
	gpio_id0 = of_get_named_gpio(dev->of_node, "panelid0-gpio", 0);
	if (gpio_id0 < 0)
		pr_err("LCM: unable to get gpio_id0\n");
	gpio_id1 = of_get_named_gpio(dev->of_node, "panelid1-gpio", 0);
	if (gpio_id1 < 0)
		pr_err("LCM: unable to get gpio_id1\n");

	pr_info("[HX8379c] GPIO ID0 %d, GPIO ID1 %d\n", gpio_id0, gpio_id1);

	/* acquaire GPIO's*/
	ret = gpio_request(gpio_id0, "gpio_id0");
	if (ret < 0)
		pr_err("LCM: unable to request gpio_id0\n");

	ret = gpio_request(gpio_id1, "gpio_id1");
	if (ret < 0)
		pr_err("LCM: unable to request gpio_id1\n");

	/* configure GPIO ID pins to input */
	ret = gpio_direction_input(gpio_id0);
	if (ret < 0)
		pr_err("LCM: unable to configure gpio_id0 as input\n");

	ret = gpio_direction_input(gpio_id1);
	if (ret < 0)
		pr_err("LCM: unable to configure  gpio_id1 as input\n");

	MDELAY(30);
	/* Read GPIO ID1 and left shift*/
	val = gpio_get_value(gpio_id1);
	vid = (val << 1);
	pr_info("[HX8379c] gpioid1 %d\n", vid);

	/* Read GPIO ID0 */
	val = gpio_get_value(gpio_id0);
	vid =  vid | val;

	pr_info("[HX8379c] panel id %d\n", vid);

	return ret;
}

/* ------------------------------------------------------------------------- */
/* lcm_probe function Implementations */
/* ------------------------------------------------------------------------- */
static int lcm_probe(struct device *dev)
{
	pr_info("[HX8379c] %s\n", __func__);
	lcm_get_vio_supply(dev);
	lcm_get_gpio(dev);
	lcm_read_panel_id(dev);
#if defined(CONFIG_PANEL_BRIGHTNESS_SCALING_DEBUG) && defined(CONFIG_PANEL_BRIGHTNESS_SCALING)
	create_brightness_scale_debug_file(dev);
#endif
	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 1);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* lcm_ids Implementations */
/* ------------------------------------------------------------------------- */
static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};


/* ------------------------------------------------------------------------- */
/* lcm_driver structure */
/* ------------------------------------------------------------------------- */
static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

/* ------------------------------------------------------------------------- */
/* lcm_drv_init function Implementations */
/* ------------------------------------------------------------------------- */
static int __init lcm_drv_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	pr_info("[HX8379c] %s\n", __func__);
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* lcm_drv_exit function Implementations */
/* ------------------------------------------------------------------------- */
static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("amazon");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

#if 0
/* ------------------------------------------------------------------------ */
/* Function: init_lcm_registers_boe */
/* ------------------------------------------------------------------------ */
static void init_lcm_registers_boe(void)
{
	unsigned int data_array[16];

	/* HX8379C for BOE 3.1" initial setting */
	data_array[0] = 0x00043902;
	data_array[1] = 0x7983ffb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00113902;
	data_array[1] = 0x1C1C44b1;
	data_array[2] = 0xD0905737;
	data_array[3] = 0x388058E2;
	data_array[4] = 0x3433F838;
	data_array[5] = 0x42;
	dsi_set_cmdq(data_array, 6, 1);

	data_array[0] = 0x000A3902;
	data_array[1] = 0x0C1480b2;
	data_array[2] = 0x11502030;
	data_array[3] = 0x00001D42;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x000B3902;
	data_array[1] = 0x01AA01b4;
	data_array[2] = 0x10AF01AF;
	data_array[3] = 0x00EA1CEA;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x00053902;
	data_array[1] = 0x000000C7;
	data_array[2] = 0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x02cc1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x77D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00263902;
	data_array[1] = 0x000700d3;
	data_array[2] = 0x08080000;
	data_array[3] = 0x00011032;
	data_array[4] = 0x03720301;
	data_array[5] = 0x00080072;
	data_array[6] = 0x05333308;
	data_array[7] = 0x05053705;
	data_array[8] = 0x00000A37;
	data_array[9] = 0x01000A00;
	data_array[10] = 0x0E00;
	dsi_set_cmdq(data_array, 11, 1);

	data_array[0] = 0x00233902;
	data_array[1] = 0x181818d5;
	data_array[2] = 0x18181818;
	data_array[3] = 0x18191918;
	data_array[4] = 0x19181818;
	data_array[5] = 0x03000119;
	data_array[6] = 0x07040502;
	data_array[7] = 0x21222306;
	data_array[8] = 0x18181820;
	data_array[9] = 0x00000018;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x00213902;
	data_array[1] = 0x181818d6;
	data_array[2] = 0x18181818;
	data_array[3] = 0x18191918;
	data_array[4] = 0x18191918;
	data_array[5] = 0x04070618;
	data_array[6] = 0x00030205;
	data_array[7] = 0x22212001;
	data_array[8] = 0x18181823;
	data_array[9] = 0x18;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x1B1600e0;
	data_array[2] = 0x233F3630;
	data_array[3] = 0x0F0D093F;
	data_array[4] = 0x13110E18;
	data_array[5] = 0x12071411;
	data_array[6] = 0x17001813;
	data_array[7] = 0x3F36301C;
	data_array[8] = 0x0C094023;
	data_array[9] = 0x110E180F;
	data_array[10] = 0x07121113;
	data_array[11] = 0x181412;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x002C2Cb6;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000000bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002c3902;
	data_array[1] = 0x070001c1;
	data_array[2] = 0x29211810;
	data_array[3] = 0x49423931;
	data_array[4] = 0x6A615A51;
	data_array[5] = 0x8B837B72;
	data_array[6] = 0xABA39B93;
	data_array[7] = 0xC9C1BAB3;
	data_array[8] = 0xEAE4DAD3;
	data_array[9] = 0x36FEF9F2;
	data_array[10] = 0x1BC01C07;
	data_array[11] = 0x0034F101;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000001bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x110800c1;
	data_array[2] = 0x322A2219;
	data_array[3] = 0x534A433B;
	data_array[4] = 0x736B635B;
	data_array[5] = 0x948C847B;
	data_array[6] = 0xB4ACA49C;
	data_array[7] = 0xD3CBC2BB;
	data_array[8] = 0xF3EBE5DB;
	data_array[9] = 0x1A3BFFF9;
	data_array[10] = 0x4507A0B6;
	data_array[11] = 0x0037C5;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000002bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x100800c1;
	data_array[2] = 0x32292219;
	data_array[3] = 0x544B433B;
	data_array[4] = 0x726B655C;
	data_array[5] = 0x938B837C;
	data_array[6] = 0xB3ABA39B;
	data_array[7] = 0xD2CAC2BA;
	data_array[8] = 0xF1EBE2DA;
	data_array[9] = 0x310CFDF8;
	data_array[10] = 0x565B3C83;
	data_array[11] = 0x005A1E;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000000bd;
	dsi_set_cmdq(data_array, 2, 1);

	/*MDELAY(200); */
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

/* ------------------------------------------------------------------------ */
/* Function: init_lcm_registers_tianma */
/* ------------------------------------------------------------------------ */
static void init_lcm_registers_tianma(void)
{
	unsigned int data_array[16];

	/* HX8379C for Tianma 3.1" initial setting */
	data_array[0] = 0x00043902;
	data_array[1] = 0x7983ffb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00113902;
	data_array[1] = 0x1A1A44b1;
	data_array[2] = 0xD0503030;
	data_array[3] = 0x388054E4;
	data_array[4] = 0x2222F838;
	data_array[5] = 0x22;
	dsi_set_cmdq(data_array, 6, 1);

	data_array[0] = 0x000A3902;
	data_array[1] = 0x0B1480b2;
	data_array[2] = 0x11501004;
	data_array[3] = 0x00001D42;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x000B3902;
	data_array[1] = 0x01AA01b4;
	data_array[2] = 0x10AA01AA;
	data_array[3] = 0x00EA1CEA;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x00053902;
	data_array[1] = 0x000000C7;
	data_array[2] = 0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x02cc1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001E3902;
	data_array[1] = 0x000700d3;
	data_array[2] = 0x04040000;
	data_array[3] = 0x00031032;
	data_array[4] = 0x00060003;
	data_array[5] = 0x00060006;
	data_array[6] = 0x08111706;
	data_array[7] = 0x05051308;
	data_array[8] = 0x0913;
	dsi_set_cmdq(data_array, 9, 1);

	data_array[0] = 0x00233902;
	data_array[1] = 0x181818d5;
	data_array[2] = 0x18242518;
	data_array[3] = 0x18020318;
	data_array[4] = 0x18000118;
	data_array[5] = 0x18191918;
	data_array[6] = 0x18181818;
	data_array[7] = 0x18202018;
	data_array[8] = 0x18181818;
	data_array[9] = 0x00000018;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x00213902;
	data_array[1] = 0x181818d6;
	data_array[2] = 0x18252418;
	data_array[3] = 0x18010018;
	data_array[4] = 0x18030218;
	data_array[5] = 0x18181818;
	data_array[6] = 0x18191918;
	data_array[7] = 0x18202018;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x150E00e0;
	data_array[2] = 0x27383230;
	data_array[3] = 0x0D0B0744;
	data_array[4] = 0x15120F17;
	data_array[5] = 0x13081514;
	data_array[6] = 0x0E001915;
	data_array[7] = 0x38323015;
	data_array[8] = 0x0B074427;
	data_array[9] = 0x120F170D;
	data_array[10] = 0x08151415;
	data_array[11] = 0x191513;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x003F37b6;
	dsi_set_cmdq(data_array, 2, 1);

	/*MDELAY(200); */
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

/* ------------------------------------------------------------------------ */
/* Function: init_lcm_registers_inx */
/* ------------------------------------------------------------------------ */
static void init_lcm_registers_inx(void)
{
	unsigned int data_array[16];

	/* HX8379C for INX 3.1" initial setting */
	data_array[0] = 0x00043902;
	data_array[1] = 0x7983ffb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00113902;
	data_array[1] = 0x181844b1;
	data_array[2] = 0xD0503131;
	data_array[3] = 0x38809EF2;
	data_array[4] = 0x3233F838;
	data_array[5] = 0x22;
	dsi_set_cmdq(data_array, 6, 1);

	data_array[0] = 0x000A3902;
	data_array[1] = 0x0F1480b2;
	data_array[2] = 0x11B7000A;
	data_array[3] = 0x00001D42;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x000B3902;
	data_array[1] = 0xD2D2D2b4;
	data_array[2] = 0x10D2D2D2;
	data_array[3] = 0x00EA1CEA;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x00053902;
	data_array[1] = 0x000000C7;
	data_array[2] = 0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00cc1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001E3902;
	data_array[1] = 0x000000d3;
	data_array[2] = 0x1414013C;
	data_array[3] = 0x00031032;
	data_array[4] = 0x01F10103;
	data_array[5] = 0x01F401F1;
	data_array[6] = 0x054449F4;
	data_array[7] = 0x05054705;
	data_array[8] = 0x0D47;
	dsi_set_cmdq(data_array, 9, 1);

	data_array[0] = 0x00233902;
	data_array[1] = 0x002120d5;
	data_array[2] = 0x04030201;
	data_array[3] = 0x1A070605;
	data_array[4] = 0x181B1B1A;
	data_array[5] = 0x24292818;
	data_array[6] = 0x18181825;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x00000018;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x00213902;
	data_array[1] = 0x032425d6;
	data_array[2] = 0x07000102;
	data_array[3] = 0x1A040506;
	data_array[4] = 0x181B1B1A;
	data_array[5] = 0x21282918;
	data_array[6] = 0x18181820;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18;
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x0E0900e0;
	data_array[2] = 0x193F332D;
	data_array[3] = 0x0C0A0639;
	data_array[4] = 0x13110D17;
	data_array[5] = 0x12071311;
	data_array[6] = 0x08001813;
	data_array[7] = 0x3F322D0D;
	data_array[8] = 0x0A063919;
	data_array[9] = 0x100E170C;
	data_array[10] = 0x08141113;
	data_array[11] = 0x00171412;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000000bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002c3902;
	data_array[1] = 0x080001c1;
	data_array[2] = 0x27211810;
	data_array[3] = 0x473F372F;
	data_array[4] = 0x655D564F;
	data_array[5] = 0x867E766D;
	data_array[6] = 0xA69E968E;
	data_array[7] = 0xC7BFB7AF;
	data_array[8] = 0xE7DFD7CF;
	data_array[9] = 0x01FFF7EF;
	data_array[10] = 0x95EDF672;
	data_array[11] = 0x00011560;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000001bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x100800c1;
	data_array[2] = 0x2E271F17;
	data_array[3] = 0x4F473F36;
	data_array[4] = 0x6D655D56;
	data_array[5] = 0x8E867E76;
	data_array[6] = 0xAEA69E96;
	data_array[7] = 0xCEC6BEB6;
	data_array[8] = 0xEDE5DED6;
	data_array[9] = 0x4A00FCF5;
	data_array[10] = 0x5B91DE51;
	data_array[11] = 0xC03ABA;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000002bd;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x002b3902;
	data_array[1] = 0x100800c1;
	data_array[2] = 0x2E261E15;
	data_array[3] = 0x50484036;
	data_array[4] = 0x6F675F57;
	data_array[5] = 0x90888078;
	data_array[6] = 0xB2A9A199;
	data_array[7] = 0xD2CAC2BA;
	data_array[8] = 0xF1E9E1DA;
	data_array[9] = 0xAB00FFF8;
	data_array[10] = 0x60EC0F02;
	data_array[11] = 0x00E764;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x007878b6;
	dsi_set_cmdq(data_array, 2, 1);

	/*MDELAY(200); */
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

/* ------------------------------------------------------------------------ */
/* Function: lcm_panel_init */
/* ------------------------------------------------------------------------ */
static void lcm_panel_init(void)
{
	pr_info("[HX8379c] %s\n", __func__);
	switch (vid) {
	case INX:
	case INXE:
		pr_info("Display INX- %s, LK\n", __func__);
		init_lcm_registers_inx();
	break;
	case TIANMA:
		pr_info("Display TIANMA- %s, LK\n", __func__);
		init_lcm_registers_tianma();
	break;
	case BOE:
		pr_info("Display BOE- %s, LK\n", __func__);
		init_lcm_registers_boe();
	break;
	}
}
#endif

/* ------------------------------------------------------------------------- */
/* lcm_set_util_funcs Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	pr_info("[HX8379c] %s\n", __func__);
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


/* ------------------------------------------------------------------------- */
/* lcm_get_params function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_get_params(LCM_PARAMS *params)
{
	pr_info("[HX8379c] %s\n", __func__);
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_TWO_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Video mode setting */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	/* HX8379C initial porch setting VSA=4Hs, VBP=8Hs, VFP=5Hs */
	if (vid == TIANMA) {
		params->dsi.vertical_sync_active = 4;
		params->dsi.vertical_backporch = 7;
		params->dsi.vertical_frontporch = 6;
		params->dsi.PLL_CLOCK = 137;
	} else if (vid == BOE) {
		params->dsi.vertical_sync_active = 4;
		params->dsi.vertical_backporch = 8;
		params->dsi.vertical_frontporch = 5;
		params->dsi.PLL_CLOCK = 137;
	} else {
		/* For INX/INXE displays, we want 50FPS */
		params->dsi.vertical_sync_active = 3;
		params->dsi.vertical_backporch = 12;
		params->dsi.vertical_frontporch = 108;
		params->dsi.PLL_CLOCK = 157;
	}

	params->dsi.horizontal_sync_active = 100;
	params->dsi.horizontal_backporch = 100;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.HS_TRAIL = 3;
	params->dsi.CLK_TRAIL = 4;
	params->dsi.HS_PRPR = 3;

	if ((INX == vid) || (INXE == vid)) {
		/* For INX/INXE displays, we want 50FPS */
		params->dsi.horizontal_frontporch = 160;
		params->dsi.clk_lp_per_line_enable = 0;
		params->dsi.cont_clock = 1;
		params->dsi.ssc_disable = 1;
	} else {
		/* HX8379C, 2 lane, 60Hz frame rate with above porch/resolution setting, DSI=388Mbps */
		/* 60FPS for the rest*/
		params->dsi.horizontal_frontporch = 180;
		params->dsi.clk_lp_per_line_enable = 1;
		params->dsi.cont_clock = 0;
	}
}

static int __init setup_lcm_id(char *str)
{
	int id;

	pr_info("enter setup_lcm_id 1 str:%s, id:%d", str, vid);
	if (get_option(&str, &id)) {
		vid = (unsigned char)id;
	}
	pr_info("enter setup_lcm_id 2 id:%d", vid);

	return 0;
}
__setup("nt35521_id=", setup_lcm_id);

/* ------------------------------------------------------------------------- */
/* lcm_power_init function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_power_init(void)
{
	pr_info("retain lk display settings\n");
}

/* ------------------------------------------------------------------------- */
/* lcm_init function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_init(void)
{
	pr_info("[HX8379c] %s\n", __func__);
	lcm_power_init();
}
/* ------------------------------------------------------------------------- */
/* lcm_suspend function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_suspend(void)
{
	unsigned int data_array[16];

	pr_info("[HX8379c] %s\n", __func__);

#ifdef CONFIG_AMAZON_METRICS_LOG
/*	char buf[128];
	snprintf(buf, sizeof(buf), "%s:lcd:suspend=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf); */
#endif

#if 0	/* remove display OFF mode */
	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
#endif
	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
}

/* ------------------------------------------------------------------------- */
/* lcm_resume function Implementations */
/* ------------------------------------------------------------------------- */
static void lcm_resume(void)
{
	unsigned int data_array[16];

	pr_info("[HX8379c] %s start\n", __func__);

	lcm_set_pwr_n(1);
	lcm_set_pwr(1);
	lcm_set_rst(1);
	lcm_set_rst(0);
	lcm_set_rst(1);

	data_array[0] = 0x00110500; /*  Sleep OUT */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 1);
/*	lcm_panel_init(); */
}

/* ------------------------------------------------------------------------- */
/* lcm_compare_id function implementations */
/* ------------------------------------------------------------------------- */
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];

	unsigned int data_array[16];

	pr_info("[HX8379c] %s\n", __func__);
	data_array[0] = 0x00043902;
	data_array[1] = 0x7983FFB9;   /*  HX8379C for BOE 3.1" ID changed */
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);
	id = (buffer[0] << 8) | buffer[1];	/* we only need ID */

	pr_debug("read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2],
		  id);

	return (LCM_ID == id) ? 1 : 0;
}

/* ------------------------------------------------------------------------- */
/* Himax driver structure */
/* ------------------------------------------------------------------------- */
LCM_DRIVER hx8379c_wvga_dsi_vdo_lcm_drv = {
	.name = "hx8379c_dsi_wvga_vdo_rook",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
};

