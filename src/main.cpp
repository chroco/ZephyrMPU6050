/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include "mpu6050.h"
#include "console.h"

int main(void)
{
	MPU6050 mpu6050 = MPU6050();

	startConsole();
	mpu6050.begin();
	mpu6050.calcGyroOffsets(true);// TODO: figure out what this does

	while(1)
	{
		mpu6050.printConditionedImuData();
		k_sleep(K_SECONDS(1));
	}
	
	return 0;
}
