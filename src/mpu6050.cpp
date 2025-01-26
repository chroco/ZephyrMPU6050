#include "mpu6050.h"

MPU6050::MPU6050() :
	mpu6050(DEVICE_DT_GET_ONE(invensense_mpu6050)),
	rawTemp(0),
	rawAccX(0), rawAccY(0), rawAccZ(0),
	rawGyroX(0), rawGyroY(0), rawGyroZ(0),
	gyroXoffset(0), gyroYoffset(0), gyroZoffset(0),
	temp(0), 
	accX(0), accY(0), accZ(0), 
	gyroX(0), gyroY(0), gyroZ(0),
	angleGyroX(0), angleGyroY(0), angleGyroZ(0),
	angleAccX(0), angleAccY(0), angleAccZ(0),
	angleX(0), angleY(0), angleZ(0),
	interval(1), // TODO: figure out what this is supposed to be
	preInterval(0),
	accCoef(0.02f), gyroCoef(0.98f)
{
	//startTriggeredImu();
}

MPU6050::MPU6050(double aC, double gC) :
	mpu6050(DEVICE_DT_GET_ONE(invensense_mpu6050))
{
  accCoef = aC;
  gyroCoef = gC;
	startTriggeredImu();
}

K_MUTEX_DEFINE(imu_data_mutex);

sensor_value_t MPU6050::temperature;
sensor_value_t MPU6050::accel[3] = {0};
sensor_value_t MPU6050::gyro[3] = {0};

int MPU6050::getImuData(
		sensor_value_t *ptemperature, sensor_value_t *paccel, sensor_value_t *pgyro, int size)
{
	
	__ASSERT(size==SENSOR_ARRAY_SIZE, "Invalid size, got %d\n", size);

	k_mutex_lock(&imu_data_mutex, K_FOREVER);
	memcpy(ptemperature, &temperature, sizeof(sensor_value_t));	
	memcpy(paccel, accel, size * sizeof(sensor_value_t));	
	memcpy(pgyro, gyro, size * sizeof(sensor_value_t));	
	k_mutex_unlock(&imu_data_mutex);
	
	return 0;
}

int MPU6050::printImuData(void) {
//*
	printf(
		"\n[%s]:\n"
		"  temp %g Cel\n"
		"  accel %f %f %f m/s/s\n"
		"  gyro  %f %f %f rad/s\n",
		now_str(),
		sensor_value_to_double(&temperature),
		sensor_value_to_double(&accel[0]),
		sensor_value_to_double(&accel[1]),
		sensor_value_to_double(&accel[2]),
		sensor_value_to_double(&gyro[0]),
		sensor_value_to_double(&gyro[1]),
		sensor_value_to_double(&gyro[2])
	);
//*/

	return 0;
}

int MPU6050::printImuData(
		sensor_value_t *ptemperature, sensor_value_t *paccel, sensor_value_t *pgyro, int size) 
{
	__ASSERT(size==SENSOR_ARRAY_SIZE, "Invalid size, got %d\n", size);
//*
	printf("\n[%s]:\n"
				 "  temp %g Cel\n"
				 "  accel %f %f %f m/s/s\n"
				 "  gyro  %f %f %f rad/s\n",
				 now_str(),
				 sensor_value_to_double(ptemperature),
				 sensor_value_to_double(&paccel[0]),
				 sensor_value_to_double(&paccel[1]),
				 sensor_value_to_double(&paccel[2]),
				 sensor_value_to_double(&pgyro[0]),
				 sensor_value_to_double(&pgyro[1]),
				 sensor_value_to_double(&pgyro[2]));
//*/

	return 0;
}

int MPU6050::process_mpu6050(const struct device *dev)
{
	
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) 
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) 
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) 
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
	} 

	return rc;
}

static struct sensor_trigger trigger;

void MPU6050::handle_mpu6050_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	int rc = process_mpu6050(dev);
	
	if (rc != 0) 
	{
		//printf("\n(%d)\n",rc);
		//printf("failure detected: %d\n", rc);
		//(void)sensor_trigger_set(dev, trig, NULL);
		//(void)sensor_trigger_set(dev, trig, handle_mpu6050_drdy);
	} 
}

int MPU6050::startTriggeredImu(void)
{
	while (!device_is_ready(mpu6050)) 
	{
		printf("Device %s is not ready\n", mpu6050->name);
		k_msleep(1000);
	}

	trigger = (struct sensor_trigger)
	{
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	int ret = 0;
	ret = sensor_trigger_set(mpu6050, &trigger, handle_mpu6050_drdy);
	__ASSERT(ret >= 0, "ERROR %d:Cannot configure trigger!", ret);

	printk("Configured for triggered sampling.\n");

	// triggered runs with its own thread after exit 
	return 0;
}

const char *MPU6050::now_str(void) 
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

void MPU6050::begin()
{
  this->update();
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
//  preInterval = millis();
}

void MPU6050::setGyroOffsets(double x, double y, double z)
{
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter)
{
	double x = 0; 
	double y = 0; 
	double z = 0;

  for(int i = 0; i < 3000; i++){
	 	x += gyroX / 65.5;
    y += gyroY / 65.5;
    z += gyroZ / 65.5;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;
//*/
}

void MPU6050::captureRaw(raw_imu_data_t *data)
{
	k_mutex_lock(&imu_data_mutex, K_FOREVER);
	memcpy(&data->temperature, &temperature, sizeof(sensor_value_t));	
	memcpy(&data->accel, accel, SENSOR_ARRAY_SIZE * sizeof(sensor_value_t));	
	memcpy(&data->gyro, gyro, SENSOR_ARRAY_SIZE * sizeof(sensor_value_t));	
	k_mutex_unlock(&imu_data_mutex);
}

void MPU6050::conditionRaw(raw_imu_data_t *raw_imu_data)
{
  temp = sensor_value_to_double(&raw_imu_data->temperature);
  //temp = (sensor_value_to_double(&raw_temperature) + 12412.0) / 340.0;
  //temp = (rawTemp + 12412.0) / 340.0;

  ///2g = 16384
	//printf("(%0.5f, %0.5f, %0.5f)\n", accX, accY, accZ);
  
	accX = sensor_value_to_double(&raw_imu_data->accel[0]) / 16384.0;
  accY = sensor_value_to_double(&raw_imu_data->accel[1]) / 16384.0;
  accZ = sensor_value_to_double(&raw_imu_data->accel[2]) / 16384.0;

	//printf(" [%0.5f, %0.5f, %0.5f] ", accX, accY, accZ);
  
	///8g = 4096
  //accX = sensor_value_to_double(&raw_accel[0]) / 4096.0;
  //accY = sensor_value_to_double(&raw_accel[1]) / 4096.0;
  //accZ = sensor_value_to_double(&raw_accel[2]) / 4096.0;

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;
  
	gyroX = sensor_value_to_double(&raw_imu_data->gyro[0]) / 65.5;
  gyroY = sensor_value_to_double(&raw_imu_data->gyro[1]) / 65.5;
  gyroZ = sensor_value_to_double(&raw_imu_data->gyro[2]) / 65.5;
	
  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;
	
	//printf(" [%0.5f, %0.5f, %0.5f] ", gyroX, gyroY, gyroZ);

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

	//printf(" [%0.5f, %0.5f, %0.5f] ", angleGyroX, angleGyroY, angleGyroZ);
  
	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

}

//*
void MPU6050::update()
{
	raw_imu_data_t imu_data = {0};
	captureRaw(&imu_data);
	conditionRaw(&imu_data);
}
//*/
/*
void MPU6050::update()
{
	static sensor_value_t raw_temperature;
	static sensor_value_t raw_accel[SENSOR_ARRAY_SIZE];
	static sensor_value_t raw_gyro[SENSOR_ARRAY_SIZE];
	
	//printf("------------------------------------------\n");
	getImuData(&raw_temperature, raw_accel, raw_gyro, SENSOR_ARRAY_SIZE);
	//printImuData(&raw_temperature, raw_accel, raw_gyro, SENSOR_ARRAY_SIZE);

  temp = sensor_value_to_double(&raw_temperature);
  //temp = (sensor_value_to_double(&raw_temperature) + 12412.0) / 340.0;
  //temp = (rawTemp + 12412.0) / 340.0;

  ///2g = 16384
	//printf("(%0.5f, %0.5f, %0.5f)\n", accX, accY, accZ);
  
	accX = sensor_value_to_double(&raw_accel[0]) / 16384.0;
  accY = sensor_value_to_double(&raw_accel[1]) / 16384.0;
  accZ = sensor_value_to_double(&raw_accel[2]) / 16384.0;

	//printf(" [%0.5f, %0.5f, %0.5f] ", accX, accY, accZ);
  
	///8g = 4096
  //accX = sensor_value_to_double(&raw_accel[0]) / 4096.0;
  //accY = sensor_value_to_double(&raw_accel[1]) / 4096.0;
  //accZ = sensor_value_to_double(&raw_accel[2]) / 4096.0;

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;
  
	gyroX = sensor_value_to_double(&raw_gyro[0]) / 65.5;
  gyroY = sensor_value_to_double(&raw_gyro[1]) / 65.5;
  gyroZ = sensor_value_to_double(&raw_gyro[2]) / 65.5;
	
  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;
	
	//printf(" [%0.5f, %0.5f, %0.5f] ", gyroX, gyroY, gyroZ);

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

	//printf(" [%0.5f, %0.5f, %0.5f] ", angleGyroX, angleGyroY, angleGyroZ);
  
	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;
}
//*/

void MPU6050::printConditionedImuData(void)
{
	//printf(" (%+5.7f) (%+5.7f, %+5.7f, %+5.7f) (%+5.7f, %+5.7f, %+5.7f)", 
	printf(" (%.2f) (%+.7f, %+.7f, %+.7f) (%+.7f, %+.7f, %+.7f)", 
			temp, accX, accY, accZ, angleX, angleY, angleZ);
	//printf("(%0.5f) (%0.5f, %0.5f, %0.5f) (%0.5f, %0.5f, %0.5f)\n", 
	//temp, accX, accY, accZ, gyroX, gyroY, gyroZ);
}                              
                               
