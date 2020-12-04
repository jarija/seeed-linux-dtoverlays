// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456.h - IIO driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * SPI is not supported by driver
 * BMA456: 7-bit I2C slave address 0x45
 */

#ifndef __MIPI_DSI_H__
#define __MIPI_DSI_H__


#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>



//#define DBG_PRINT(format, x...)	printk(KERN_INFO "[TST]%d:%s " format, __LINE__, __func__, ##x)
#define DBG_FUNC(format, x...)		printk(KERN_INFO "[DSI]%s:" format"\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[DSI]" format"\n", ##x)


#endif /*End of header guard macro */
