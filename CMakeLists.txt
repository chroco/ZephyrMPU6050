# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_custom_target)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
target_sources(app 
	PRIVATE 
		src/console.cpp
		src/mpu6050.cpp
		src/main.cpp
)
