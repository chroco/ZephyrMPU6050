#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/timing/timing.h>
#include <stdio.h>
#include <math.h>

#define SENSOR_ARRAY_SIZE 3
#define PI 3.14159265358979323846

typedef struct sensor_value sensor_value_t;
typedef uint8_t byte;

struct raw_imu_data_t
{
	sensor_value_t temperature;
	sensor_value_t accel[SENSOR_ARRAY_SIZE];
	sensor_value_t gyro[SENSOR_ARRAY_SIZE];
};

class MPU6050{
  public:
		MPU6050();
		MPU6050(double aC, double gC);

		void begin();

		const char *now_str(void);
		int printImuData(sensor_value_t  *, sensor_value_t *, sensor_value_t *, int);
		int printImuData(void);
		int getImuData(sensor_value_t  *, sensor_value_t *, sensor_value_t *, int);

		void update();
		
		void setGyroOffsets(double x, double y, double z);
		double getYawOffset(){ return gyroZoffset;}

		int16_t getRawAccX(){ return rawAccX; };
		int16_t getRawAccY(){ return rawAccY; };
		int16_t getRawAccZ(){ return rawAccZ; };

		int16_t getRawTemp(){ return rawTemp; };

		int16_t getRawGyroX(){ return rawGyroX; };
		int16_t getRawGyroY(){ return rawGyroY; };
		int16_t getRawGyroZ(){ return rawGyroZ; };

		double getTemp(){ return temp; };

		double getAccX(){ return accX; };
		double getAccY(){ return accY; };
		double getAccZ(){ return accZ; };

		double getGyroX(){ return gyroX; };
		double getGyroY(){ return gyroY; };
		double getGyroZ(){ return gyroZ; };

		void calcGyroOffsets(
				bool console = false, 
				uint16_t delayBefore = 1000, 
				uint16_t delayAfter = 3000
		);

		double getGyroXoffset(){ return gyroXoffset; };
		double getGyroYoffset(){ return gyroYoffset; };
		double getGyroZoffset(){ return gyroZoffset; };

		double getAccAngleX(){ return angleAccX; };
		double getAccAngleY(){ return angleAccY; };

		double getGyroAngleX(){ return angleGyroX; };
		double getGyroAngleY(){ return angleGyroY; };
		double getGyroAngleZ(){ return angleGyroZ; };

		double getAngleX(){ return angleX; };
		double getAngleY(){ return angleY; };
		double getAngleZ(){ return angleZ; };

		void printConditionedImuData(void);
  private:
		static sensor_value_t temperature;
		static sensor_value_t accel[SENSOR_ARRAY_SIZE];
		static sensor_value_t gyro[SENSOR_ARRAY_SIZE];
		
		const struct device *const mpu6050;

		void captureRaw(raw_imu_data_t *);
		void conditionRaw(raw_imu_data_t *);

		int startTriggeredImu(void);
		static void handle_mpu6050_drdy(const struct device *, const struct sensor_trigger *);
		static int process_mpu6050(const struct device *);

		int16_t rawTemp;
		int16_t rawAccX, rawAccY, rawAccZ; 
		int16_t rawGyroX, rawGyroY, rawGyroZ;

		double gyroXoffset, gyroYoffset, gyroZoffset;

		double temp; 
		double accX, accY, accZ, gyroX, gyroY, gyroZ;

		double angleGyroX, angleGyroY, angleGyroZ;
		double angleAccX, angleAccY, angleAccZ;

		double angleX, angleY, angleZ;

		double interval;
		long preInterval;

		double accCoef, gyroCoef;
};

#endif
