/*
 * Mpu9250.cpp
 *
 *  Created on: Aug 6, 2020
 *      Author: alexandr
 */

#include "Mpu9250.h"

Mpu9250::Mpu9250() {

	SPI_t &mySPI = vspi;  // vspi and hspi are the default objects

}

Mpu9250::~Mpu9250() {
}

void Mpu9250::init() {
	spi_device_handle_t device;
	ESP_ERROR_CHECK( mySPI.begin(23, 19, 18));
	ESP_ERROR_CHECK( mySPI.addDevice(0, 1000000, 5, &device));

    uint8_t am;
    mySPI.readBytes(device,0x75,1,&am);
    printf("%02hx \r\n",am);
    lsb_to_mg_accel = 2000.0f / 32768.0f;
}

void Mpu9250::write_reg(uint8_t reg, uint8_t data) {
	mySPI.writeByte(device, reg, data);
}

uint8_t Mpu9250::read_reg(uint8_t reg) {
	uint8_t rx;
	mySPI.readBytes(device, reg, 1, &rx);
	return rx;
}

void Mpu9250::read_raw_data() {
    ESP_ERROR_CHECK(mySPI.readBytes(device, 0x3B, 14, rx_buffer));
    raw_data_accel[0] = (rx_buffer[1]) | rx_buffer[0] << 8;
    raw_data_accel[1] = (rx_buffer[3]) | rx_buffer[2] << 8;
    raw_data_accel[2] = (rx_buffer[5]) | rx_buffer[4] << 8;

    raw_temp = (rx_buffer[7]) | rx_buffer[6] << 8;

    raw_data_gyro[0] = (rx_buffer[9]) | rx_buffer[8] << 8;
    raw_data_gyro[1] = (rx_buffer[11]) | rx_buffer[10] << 8;
    raw_data_gyro[2] = (rx_buffer[13]) | rx_buffer[12] << 8;
}

void Mpu9250::read_data() {
	read_raw_data();
	accel = raw_data_accel;
	anguar_velo = raw_data_gyro;
	mag = raw_data_mag;
	accel *= lsb_to_mg_accel;
	anguar_velo *= lsb_to_dps_gyro;
	mag *= lsb_to_nT_mag;


}
