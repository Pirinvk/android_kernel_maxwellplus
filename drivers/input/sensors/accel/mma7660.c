/* drivers/input/sensors/access/mma7660.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/mma7660.h>
#if 0
#define SENSOR_DEBUG_TYPE SENSOR_TYPE_ACCEL
#define DBG(x...) if(sensor->pdata->type == SENSOR_DEBUG_TYPE) printk(x)
#else
#define DBG(x...)
#endif

#define MMA7660_ENABLE		1

#define MMA7660_REG_X_OUT       0x0
#define MMA7660_REG_Y_OUT       0x1
#define MMA7660_REG_Z_OUT       0x2
#define MMA7660_REG_TILT        0x3
#define MMA7660_REG_SRST        0x4
#define MMA7660_REG_SPCNT       0x5
#define MMA7660_REG_INTSU       0x6
#define MMA7660_REG_MODE        0x7
#define MMA7660_REG_SR          0x8
#define MMA7660_REG_PDET        0x9
#define MMA7660_REG_PD          0xa


#define MMA7660_RANGE			1500000

/* LIS3DH */
#define MMA7660_PRECISION       6
#define MMA7660_BOUNDARY		(0x1 << (MMA7660_PRECISION - 1))
#define MMA7660_GRAVITY_STEP		(MMA7660_RANGE / MMA7660_BOUNDARY)

#define MMA7660_COUNT_AVERAGE	1

#define G_CONST             990000
#define MMA7660_SPEED		200 * 1000
#define NR_SAMPHISTLEN 5
#define NR_CHECK_NUM (10*NR_SAMPHISTLEN)
#define CABLIC_NUM 10
#define HEAD 0
#define END  (NR_SAMPHISTLEN)
#define SORT 0

#define CABLIC_FILE   "/data/mma7660_cablic.dat"
#if defined(CONFIG_BQ_C7011)
static const char def_arr[] = {
	0x63, 0x0D, 0x00, 0x00, 0x25, 0x14, 0x00, 0x00, 0xA1, 0xAB, 0x00, 0x00, 0x32, 0xC9, 0xFF, 0xFF,
	0xD4, 0xEB, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0xD7, 0x0B, 0x00, 0x00, 0x56, 0x18, 0x00, 0x00, 0xC9, 0x95, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x27, 0xF5, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xCE, 0x0A, 0x00, 0x00, 0x54, 0x1A, 0x00, 0x00,
	0x85, 0x96, 0x00, 0x00, 0x0B, 0xC9, 0xFF, 0xFF, 0x80, 0xF8, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xDA, 0x07, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0xE8, 0xFF, 0xFF, 0x29, 0xD8, 0xFF, 0xFF,
	0x4F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x9B, 0x0D, 0x00, 0x00, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xCB, 0xFF, 0xFF,
	0x8D, 0xC7, 0xFF, 0xFF, 0x7F, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0xAD, 0x14, 0x00, 0x00, 0x5B, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x71, 0xE9, 0xFF, 0xFF, 0x70, 0xE8, 0xFF, 0xFF, 0x4F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x16, 0x00, 0x00, 0x15, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xE4, 0xFF, 0xFF, 0x15, 0x66, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xBB, 0x09, 0x00, 0x00,
	0x35, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0xF8, 0xFF, 0xFF, 0x49, 0xF4, 0xFF, 0xFF,
	0x15, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x25, 0x0C, 0x00, 0x00, 0xC1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED, 0xFF, 0xFF, 0xFF,
	0xCE, 0xDC, 0xFF, 0xFF, 0x15, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x5B, 0x08, 0x00, 0x00, 0x4D, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFD, 0xEC, 0xFF, 0xFF, 0xC9, 0xCE, 0xFF, 0xFF, 0xDC, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
};

#else
static const char def_arr[] = {
0xA2, 0x8F, 0x00, 0x00, 0x55, 0x22, 0x00, 0x00, 0xCC, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x9A, 0xFF, 0xFF,
0xA4, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x25, 0x40, 0x00, 0x00, 0x71, 0x35, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x89, 0xD4, 0xFF, 0xFF, 0x3C, 0xB6, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x01, 0x00, 0x00, 0x00, 0x18, 0x1F, 0x00, 0x00, 0x20, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1, 0xD7, 0xFF, 0xFF,
0xFA, 0xE1, 0xFF, 0xFF, 0x71, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
#endif
struct cablic {
	int xoffset;
	int yoffset;
	int zoffset;
	int xoffset1;
	int yoffset1;
	int zoffset1;
	int num1x;
	int num1y;
	int num1z;
	int num2x;
	int num2y;
	int num2z;
	int valid;
};

struct sensor_axis_average {
		int x_average;
		int y_average;
		int z_average;
		int count;
};

static int xoffset1 = 0;
static int yoffset1 = 0;
static int zoffset1 = 0;
static int num1x = 0;
static int num1y = 0;
static int num1z = 0;
static int xoffset2 = 0;
static int yoffset2 = 0;
static int zoffset2 = 0;
static int num2x = 0;
static int num2y = 0;
static int num2z = 0;

static struct sensor_axis_average axis_average;
static struct cablic cab_arry[CABLIC_NUM];
static int current_num = 0;
static int revision = -1;
static long int last_z =0, last_x = 0, last_y =0;
static int gsensor_check = 0;
static int gsensor_get = 0;
static int gsensor_check_num = 0;
static struct kobject *android_gsensor_kobj;

struct mma7660_axis mma_dejitter[NR_SAMPHISTLEN];
struct mma7660_axis_w mma_sort[NR_SAMPHISTLEN];
static const unsigned char weight [NR_SAMPHISTLEN ] =
{
	   6, 4, 3, 3 /* The last element is pow2(SUM(0..3)) */
};
/* AKM HW info */
static void mma7660_average (struct mma7660_axis *samp)
{
	int liTmp;
	long int  x = 0, y = 0, z = 0, Xsum = 0,Ysum = 0, Zsum = 0,diff;

	for (liTmp = NR_SAMPHISTLEN-1; liTmp > 0; liTmp--)
	{
		mma_dejitter[liTmp].x = mma_dejitter[liTmp-1].x;
		mma_dejitter[liTmp].y = mma_dejitter[liTmp-1].y;
		mma_dejitter[liTmp].z = mma_dejitter[liTmp-1].z;
	}

	mma_dejitter[0].x = samp->x;
	mma_dejitter[0].y = samp->y;
	mma_dejitter[0].z = samp->z;

 #if SORT
// sort num
	for (liTmp = 0; liTmp < NR_SAMPHISTLEN; liTmp++)
	{
		mma_sort[liTmp].x = mma_dejitter[liTmp].x;
		mma_sort[liTmp].y = mma_dejitter[liTmp].y;
		mma_sort[liTmp].z = mma_dejitter[liTmp].z;
		mma_sort[liTmp].xw =mma_sort[liTmp].yw = mma_sort[liTmp].zw= weight[liTmp];
	}

	for(i=0;i<NR_SAMPHISTLEN-1;i++)
		for(j=0;j<NR_SAMPHISTLEN-i-1;j++)
		{
			if(mma_sort[j].x > mma_sort[j+1].x)
			{
				t.v =mma_sort[j].x;
				t.w = mma_sort[j].xw;
				mma_sort[j].x = mma_sort[j+1].x;
				mma_sort[j].xw = mma_sort[j+1].xw;
				mma_sort[j+1].x = t.v;
				mma_sort[j+1].xw = t.w;
			}

			if(mma_sort[j].y > mma_sort[j+1].y)
			{
				t.v = mma_sort[j].y;
				t.w = mma_sort[j].yw;
				mma_sort[j].y = mma_sort[j+1].y;
				mma_sort[j].yw = mma_sort[j+1].yw;
				mma_sort[j+1].y = t.v;
				mma_sort[j+1].yw = t.w;
			}

			if(mma_sort[j].z > mma_sort[j+1].z)
			{
				t.v=mma_sort[j].z;
				t.w = mma_sort[j].zw;
				mma_sort[j].z = mma_sort[j+1].z;
				mma_sort[j].zw = mma_sort[j+1].zw;
				mma_sort[j+1].z = t.v;
				mma_sort[j+1].zw = t.w;
			}
		}

	for (liTmp = HEAD; liTmp < END; liTmp++)
	{
		x += mma_sort[liTmp].x * mma_sort[liTmp].xw;
		y += mma_sort[liTmp].y * mma_sort[liTmp].yw;
		z += mma_sort[liTmp].z * mma_sort[liTmp].zw;
		Xsum += mma_sort[liTmp].xw;
		Ysum += mma_sort[liTmp].yw;
		Zsum += mma_sort[liTmp].zw;
	}

 #else

	for (liTmp = 0; liTmp < NR_SAMPHISTLEN; liTmp++)
	{
		mma_sort[liTmp].xw =mma_sort[liTmp].yw = mma_sort[liTmp].zw= weight[liTmp];
	}

	for (liTmp = HEAD; liTmp < END; liTmp++)
	{
		x += mma_dejitter[liTmp].x * mma_sort[liTmp].xw;
		y += mma_dejitter[liTmp].y * mma_sort[liTmp].yw;
		z += mma_dejitter[liTmp].z * mma_sort[liTmp].zw;
		Xsum += mma_sort[liTmp].xw;
		Ysum += mma_sort[liTmp].yw;
		Zsum += mma_sort[liTmp].zw;
	}

 #endif

	samp->x = x / Xsum;
	samp->y = y /Ysum;
	samp->z = z /Zsum;


	diff = ((samp->x - last_x) > 0)?(samp->x - last_x):-(samp->x - last_x);
#if 0
	if(diff < 46876)
	{
		samp->x  = 	last_x;
	}
	else
	{
		last_x = samp->x;
	}

	diff = ((samp->y - last_y) > 0)?(samp->y - last_y):-(samp->y - last_y);
	if(diff < 46876)
	{
		samp->y  = 	last_y;
	}
	else
	{
		last_y = samp->y;
	}
#endif
	diff = ((samp->z - last_z) > 0)?(samp->z - last_z):-(samp->z - last_z);
	if(diff < 46876)
	{
		samp->z  = 	last_z;
	}
	else
	{
		last_z = samp->z;
	}


}


static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%#x\n", revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static int mma7660_load_cablic(const char *addr)
{
	int ret;
	long fd = sys_open(addr,O_RDONLY,0);

	if(fd < 0){
		printk("mma7660_load_offset: open file %s\n", CABLIC_FILE);
		return -1;
	}
	ret = sys_read(fd,(char __user *)cab_arry,sizeof(cab_arry));
	sys_close(fd);

	return ret;
}

static void mma7660_put_cablic(const char *addr)
{
	char value[128];
	long fd = sys_open(addr,O_CREAT | O_RDWR | O_TRUNC,0);

	if(fd<0){
		printk("mma7660_put_offset: open file %s\n", CABLIC_FILE);
		return;
	}
	sys_write(fd,(const char __user *)cab_arry,sizeof(cab_arry));

	sys_close(fd);
}
static ssize_t gsensor_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t gsensor_check_store(struct device *dev,
		struct device_attribute *attr, char *buf,size_t count)
{
	char buffer[3];
	int num=20;

	gsensor_check = 1;
	xoffset1 = 0;
	yoffset1 = 0;
	zoffset1 = 0;
	num1x = 0;
	num1y = 0;
	num1z = 0;
	xoffset2 = 0;
	yoffset2 = 0;
	zoffset2 = 0;
	num2x = 0;
	num2y = 0;
	num2z = 0;
	return count;

}
static DEVICE_ATTR(gsensorcheck, 0666, gsensor_check_show, gsensor_check_store);

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		DBG(KERN_ERR
		       "MMA7660 gsensor_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);
	if (ret) {
		DBG(KERN_ERR
		       "MMA7660 gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensorcheck.attr);   // "gsensorcheck"
	if (ret) {
		DBG(KERN_ERR
		       "mma7660 gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}
	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static void mma7660_get_offset_data(void){
	if (gsensor_get){
		gsensor_get = 0;
		mma7660_load_cablic(CABLIC_FILE);
	}
	return;
}

/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;

	gsensor_get = 1;
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	
	//register setting according to chip datasheet		
	if(enable)
	{	
		status = MMA7660_ENABLE;	//mma7660
		sensor->ops->ctrl_data |= status;	
	}
	else
	{
		status = ~MMA7660_ENABLE;	//mma7660
		sensor->ops->ctrl_data &= status;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	return result;

}

static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;

	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_OFF;

	DBG("%s:MMA7660_REG_TILT=0x%x\n",__func__,sensor_read_reg(client, MMA7660_REG_TILT));

	result = sensor_write_reg(client, MMA7660_REG_SR, (0x01<<5)| 0x02);	//32 Samples/Second Active and Auto-Sleep Mode
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	if(sensor->pdata->irq_enable)	//open interrupt
	{
		result = sensor_write_reg(client, MMA7660_REG_INTSU, 1<<4);//enable int,GINT=1
		if(result)
		{
			printk("%s:line=%d,error\n",__func__,__LINE__);
			return result;
		}
	}
	
	sensor->ops->ctrl_data = 1<<6;	//Interrupt output INT is push-pull
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	memset(&axis_average, 0, sizeof(struct sensor_axis_average));

	return result;
}


static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
    s64 result;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	//int precision = sensor->ops->precision;
		
	result = (int)low_byte;
	if (result < MMA7660_BOUNDARY)
		result = result* MMA7660_GRAVITY_STEP;
	else
		result = ~( ((~result & (0x7fff>>(16-MMA7660_PRECISION)) ) + 1) 
	   			* MMA7660_GRAVITY_STEP) + 1;    

    	return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

#define GSENSOR_MIN  		2
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x,y,z;
	int i;
	struct sensor_axis axis;
	char buffer[3] = {0};	
	char value = 0;
	
	if(sensor->ops->read_len < 3)	//sensor->ops->read_len = 3
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	memset(buffer, 0, 3);
	
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	mma7660_get_offset_data();
	//this gsensor need 6 bytes buffer
	axis.x = sensor_convert_data(sensor->client, 0, buffer[0]);	//buffer[1]:high bit
	axis.y = sensor_convert_data(sensor->client, 0, buffer[1]);
	axis.z = sensor_convert_data(sensor->client, 0, buffer[2]);

	mma7660_average (&axis);

	for (i = 0; i < CABLIC_NUM; i++) {
		if (cab_arry[i].valid == 0) break;
		if (axis.x > 0)
			axis.x = axis.x - cab_arry[i].xoffset;
		else
			axis.x = axis.x - cab_arry[i].xoffset1;

		if (axis.y > 0)
			axis.y = axis.y - cab_arry[i].yoffset;
		else
			axis.y = axis.y - cab_arry[i].yoffset1;

		if (axis.z > G_CONST)
			axis.z = axis.z - cab_arry[i].zoffset;
		else
			axis.z = axis.z + cab_arry[i].zoffset1;
	}

	if (gsensor_check){
		if (gsensor_check_num++ < NR_CHECK_NUM) {
			if (axis.x > 0) {
				xoffset1 += axis.x;
				num1x++;
			} else {
				xoffset2 += axis.x;
				num2x++;
			}

			if (axis.y > 0) {
				yoffset1 += axis.y;
				num1y++;
			} else {
				yoffset2 += axis.y;
				num2y++;
			}

			if (axis.z > G_CONST) {
				zoffset1 += (axis.z - G_CONST);
				num1z++;
			} else {
				zoffset2 += (G_CONST - axis.z);
				num2z++;
			}
		} else {
			gsensor_check = 0;
			gsensor_check_num = 0;
			if (num1x > 0)
				xoffset1 = xoffset1/num1x;
			else
				xoffset1 = 0;

			if (num1y > 0)
				yoffset1 = yoffset1/num1y;
			else
				yoffset1 = 0;

			if (num1z > 0)
				zoffset1 = zoffset1/num1z;
			else
				zoffset1 = 0;

			if (num2x > 0)
				xoffset2 = xoffset2/num2x;
			else
				xoffset2 = 0;

			if (num2y > 0)
				yoffset2 = yoffset2/num2y;
			else
				yoffset2 = 0;

			if (num2z > 0)
				zoffset2 = zoffset2/num2z;
			else
				zoffset2 = 0;

			printk("num1x=%d,num2x=%d,num1y=%d,num2y=%d,num1z=%d,num2z=%d\n",num1x, num2x, num1y, num2y, num1z, num2z);
			printk("xoffset1=%d,yoffset1=%d,zoffset1=%d\n",xoffset1,yoffset1,zoffset1);
			printk("xoffset2=%d,yoffset2=%d,zoffset2=%d\n",xoffset2,yoffset2,zoffset2);
			if (current_num == CABLIC_NUM) current_num = 0;
			cab_arry[current_num].xoffset  = xoffset1;
			cab_arry[current_num].xoffset1 = xoffset2;
			cab_arry[current_num].yoffset  = yoffset1;
			cab_arry[current_num].yoffset1 = yoffset2;
			cab_arry[current_num].zoffset  = zoffset1;
			cab_arry[current_num].zoffset1 = zoffset2;
			cab_arry[current_num++].valid  = 1;
			mma7660_put_cablic(CABLIC_FILE);
		}
	}

	x = axis.x;
	y = axis.y;
	z = axis.z;
	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	axis_average.x_average += axis.x;
	axis_average.y_average += axis.y;
	axis_average.z_average += axis.z;
	axis_average.count++;

	if(axis_average.count >= MMA7660_COUNT_AVERAGE)
	{
		axis.x = axis_average.x_average / axis_average.count;
		axis.y = axis_average.y_average / axis_average.count;
		axis.z = axis_average.z_average / axis_average.count;

		DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

		memset(&axis_average, 0, sizeof(struct sensor_axis_average));

		//Report event only while value is changed to save some power
		if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
		{
			gsensor_report_value(client, &axis);

			/* ?£¤3a¦Ì??o¡ä?¨ºy?Y. */
			mutex_lock(&(sensor->data_mutex) );
			sensor->axis = axis;
			mutex_unlock(&(sensor->data_mutex) );
		}
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
	
	return ret;
}


struct sensor_operate gsensor_mma7660_ops = {
	.name				= "mma7660",
	.type				= SENSOR_TYPE_ACCEL,			//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_MMA7660_l,			//i2c id number
	.read_reg			= MMA7660_REG_X_OUT,			//read data
	.read_len			= 3,					//data length
	.id_reg				= SENSOR_UNKNOW_DATA,			//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,			//device id
	.precision			= MMA7660_PRECISION,			//12 bit
	.ctrl_reg 			= MMA7660_REG_MODE,			//enable or disable 	
	.int_status_reg 		= SENSOR_UNKNOW_DATA,			//intterupt status register
	.range				= {-MMA7660_RANGE,MMA7660_RANGE},	//range
	.trig				= IRQF_TRIGGER_LOW|IRQF_ONESHOT,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report 			= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_mma7660_ops;
}


static int __init gsensor_mma7660_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);	
	DBG("%s\n",__func__);
	memset(cab_arry, 0x00, sizeof(cab_arry));
	memcpy((char*)cab_arry, def_arr, sizeof(cab_arry));
	result = gsensor_sysfs_init();
	return result;
}

static void __exit gsensor_mma7660_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_mma7660_init);
module_exit(gsensor_mma7660_exit);



