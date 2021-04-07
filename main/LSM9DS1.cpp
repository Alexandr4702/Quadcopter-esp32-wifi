/*
 * LSM9DS1.cpp
 *
 *  Created on: Apr 4, 2021
 *      Author: gilg
 */

#include "LSM9DS1.h"

LSM9DS1::LSM9DS1(spi_host_device_t spi_bus_, int mosi_pin, int miso_pin, int sck_pin,int cs_pin_accel_gyro, int cs_pin_mag): spi_bus(spi_bus_)
{
	spi_bus.begin(mosi_pin, miso_pin, sck_pin, 1000);
	spi_bus.addDevice(0, 8000000, cs_pin_accel_gyro, &device_accel_gyro);
	spi_bus.addDevice(0, 8000000, cs_pin_mag, &device_mag);
}

LSM9DS1::~LSM9DS1() {
	// TODO Auto-generated destructor stub
}

void LSM9DS1::read_raw_data() {
	spi_bus.readBytes(device_accel_gyro, OUT_TEMP_L, 9, RAW_GYRO_TEMPERATURE);
	spi_bus.readBytes(device_accel_gyro, STATUS_REG_1, 7, RAW_ACCEL);
	spi_bus.readBytes(device_mag, STATUS_REG_M | 0x4 , 7, RAW_MAG);
//	spi_bus.readByte(device_mag, OUT_Y_L_M, RAW_MAG);

	TEMPERATURE_LSB = RAW_GYRO_TEMPERATURE[0] | (RAW_GYRO_TEMPERATURE[1] << 8);
	GYRO_LSB[0] = RAW_GYRO_TEMPERATURE[3] | (RAW_GYRO_TEMPERATURE[4] << 8);
	GYRO_LSB[1] = RAW_GYRO_TEMPERATURE[5] | (RAW_GYRO_TEMPERATURE[6] << 8);
	GYRO_LSB[2] = RAW_GYRO_TEMPERATURE[7] | (RAW_GYRO_TEMPERATURE[8] << 8);

//	printf("%02hx %5hi %5hi %5hi %5hi \n", RAW_GYRO_TEMPERATURE[2], TEMPERATURE_LSB, GYRO_LSB[0], GYRO_LSB[1], GYRO_LSB[2]);
//	printf("gyro %02hx %02hx %02hx %02hx %02hhx %02hx %02hx %02hx %02hx %02hx\n",
//			RAW_GYRO_TEMPERATURE[0], RAW_GYRO_TEMPERATURE[1], RAW_GYRO_TEMPERATURE[2],
//			RAW_GYRO_TEMPERATURE[3], RAW_GYRO_TEMPERATURE[4], RAW_GYRO_TEMPERATURE[5],
//			RAW_GYRO_TEMPERATURE[6], RAW_GYRO_TEMPERATURE[7], RAW_GYRO_TEMPERATURE[8], RAW_GYRO_TEMPERATURE[9]
//			);
//
//	printf("accel %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx \n",
//			RAW_ACCEL[0], RAW_ACCEL[1], RAW_ACCEL[2],
//			RAW_ACCEL[3], RAW_ACCEL[4], RAW_ACCEL[5],
//			RAW_ACCEL[6]
//			);

	printf("mag %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx \n",
			RAW_MAG[0], RAW_MAG[1], RAW_MAG[2],
			RAW_MAG[3], RAW_MAG[4], RAW_MAG[5],
			RAW_MAG[6]
			);
}

void LSM9DS1::software_reset() {
	spi_bus.writeByte(device_accel_gyro, CTRL_REG8, 1);
	spi_bus.writeByte(device_mag, CTRL_REG2_M, 4);
}

int LSM9DS1::init() {
	uint8_t rx;
	spi_bus.readBytes(device_accel_gyro, WHO_AM_I_XG, 1, &rx);
	if(rx != WHO_AM_I_AG_RSP)
	{
		fprintf(stderr, "gyro unsuccessful initialization lsm9ds1 \n ");
		return -1;
	}

	spi_bus.readBytes(device_mag, WHO_AM_I_M, 1, &rx);
	if(rx != WHO_AM_I_M_RSP)
	{
		fprintf(stderr, "mag unsuccessful initialization lsm9ds1 \n ");
		return -1;
	}

	uint8_t CTRL_REG1_G_val = 0;
	uint8_t ODR_G = 3;
	uint8_t FS_G = 1;
	uint8_t BW_G = 0;
	CTRL_REG1_G_val |= (ODR_G << 5) | (FS_G << 3) | (BW_G);
	spi_bus.writeByte(device_accel_gyro, CTRL_REG1_G, CTRL_REG1_G_val);

	uint8_t CTRL_REG6_XL_val = 0;
	uint8_t ODR_XL = 3;
	uint8_t FS_XL = 2;
	uint8_t BW_SCAL_ODR = 0;
	uint8_t BW_XL = 3;
	CTRL_REG6_XL_val |= (ODR_XL << 5) | (FS_XL << 3) | (BW_SCAL_ODR << 2) | (BW_XL);
	spi_bus.writeByte(device_accel_gyro, CTRL_REG6_XL, CTRL_REG6_XL_val);

	uint8_t CTRL_REG1_M_val = 0;
	uint8_t TEMP_COMP = 0;
	uint8_t OM = 2;
	uint8_t DO = 7;
	uint8_t FAST_ODR = 1;
	uint8_t ST = 0;
	CTRL_REG1_M_val |= (ST) | (FAST_ODR << 1) | (DO << 2) | (OM << 5) | (TEMP_COMP << 7);
	spi_bus.writeByte(device_mag, CTRL_REG1_M, CTRL_REG1_M_val);

	uint8_t CTRL_REG2_M_val = 0;
	uint8_t FS = 0;
	CTRL_REG2_M_val |= (FS << 5);
	spi_bus.writeByte(device_mag, CTRL_REG2_M, CTRL_REG2_M_val);

	uint8_t CTRL_REG3_M_val = 0;
	uint8_t I2C_DISABLE = 1;
	uint8_t LP = 0;
	uint8_t SIM = 0;
	uint8_t MD = 0;
	CTRL_REG3_M_val |= (MD) | (SIM << 2) | (LP << 5) | (I2C_DISABLE << 7);
	spi_bus.writeByte(device_mag, CTRL_REG3_M, CTRL_REG3_M_val);

	uint8_t CTRL_REG4_M_val = 0;
	uint8_t OMZ = 3;
	CTRL_REG4_M_val |= (OMZ << 2);
	spi_bus.writeByte(device_mag, CTRL_REG4_M, CTRL_REG4_M_val);

	uint8_t CTRL_REG5_M_val = 0;
	spi_bus.writeByte(device_mag, CTRL_REG5_M, CTRL_REG5_M_val);

	return 0;
}
//LSM9DS1::LSM9DS1(const LSM9DS1 &other) {
//	// TODO Auto-generated constructor stub
//
//}
//
//LSM9DS1::LSM9DS1(LSM9DS1 &&other) {
//	// TODO Auto-generated constructor stub
//
//}
//
//LSM9DS1& LSM9DS1::operator=(const LSM9DS1 &other) {
//	// TODO Auto-generated method stub
//
//}
//
//LSM9DS1& LSM9DS1::operator=(LSM9DS1 &&other) {
//	// TODO Auto-generated method stub
//
//}

