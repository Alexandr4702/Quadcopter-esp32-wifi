/*
 * Mpu9250.cpp
 *
 *  Created on: Aug 6, 2020
 *      Author: alexandr
 */

#include "Mpu9250.h"

void print_bytes(void *ptr,uint16_t cnt_bytes)
{
	uint8_t* buff =reinterpret_cast<uint8_t*>(ptr);
	for(uint16_t i=0;i<cnt_bytes;i++)
	{
		printf("%02hx ",buff[i]);
	}
	printf("\r\n");
}

Mpu9250::Mpu9250() {

//	SPI_t &mySPI = vspi;  // vspi and hspi are the default objects

}

Mpu9250::~Mpu9250() {
}

#define MPU_InitRegNum 16
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F

void Mpu9250::init() {

	init_spi();




    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        //{0x80, PWR_MGMT_1},     // Reset Device - Disabled because it seems to corrupt initialisation of AK8963
        {0x01, PWR_MGMT_1},     // Clock Source
        {0x00, PWR_MGMT_2},     // Enable Acc & Gyro
        {0x01, CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, GYRO_CONFIG},    // +-2000dps
        {0x08, ACCEL_CONFIG},   // +-4G
        {0x09, ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, INT_PIN_CFG},    //
        //{0x40, I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, USER_CTRL},      // Enable AUX
        {0x20, USER_CTRL},       // I2C Master mode
        {0x0D, I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

        {AK8963_I2C_ADDR, I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, I2C_SLV4_CTRL},
        //{0x81, I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, I2C_SLV0_DO}, // Reset AK8963
        {0x81, I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

    };
    //spi.format(8,0);
    //spi.frequency(1000000);

    for(i=0; i<MPU_InitRegNum; i++) {
    	write_reg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        delay(100);  //I2C must slow down the write speed, otherwise it won't work
    }




    return ;









    //select clock source
    PWR_MGMT_1_value = 1;
    write_reg(PWR_MGMT_1, PWR_MGMT_1_value);
    //pwr down magnetometrs
    {
		//enable i2c master mode
		uint8_t I2C_MST_EN = 1;
		USER_CTRL_value = 0|I2C_MST_EN<<5;
		write_reg(USER_CTRL,USER_CTRL_value);

		//set i2c bus clock 400kHz
		uint8_t I2C_MST_CLK = 13;
		I2C_MST_CTRL_value = I2C_MST_CLK;
		write_reg(I2C_MST_CTRL,I2C_MST_CTRL_value);

	    uint8_t who_am_i = read_reg_to_AK8963(0x00);
	    printf("mag %02hx \r\n",who_am_i);
		//pwr down ak8963
		write_register_to_AK8963(CNTL1,0x00);
    }
    //reset
    write_reg(PWR_MGMT_1,0x80);
    delay(1);

    CNTL2_value = 0x01;
    write_register_to_AK8963(CNTL2,CNTL2_value);

    //select clock source
    PWR_MGMT_1_value = 1;
    write_reg(PWR_MGMT_1, PWR_MGMT_1_value);

    //checking gyro and accel valut must be equal 0x71 or 0x73
    uint8_t am = read_reg(WHO_AM_I);
    printf("%02hx \r\n",am);

    //enable sensor
    write_reg(PWR_MGMT_2,0x00);

    //-----------setting up gyro and accel-----
    {
		SMPLRT_DIV_value = 9;
		write_reg(SMPLRT_DIV, SMPLRT_DIV_value);

		uint8_t DLPF_CFG = 3;
		CONFIG_value = DLPF_CFG;
		write_reg(CONFIG, CONFIG_value);

		uint8_t Fchoice_b = 0;
		uint8_t GYRO_FS_SEL = 0x0;
		GYRO_CONFIG_value = Fchoice_b | GYRO_FS_SEL << 3;
		write_reg(GYRO_CONFIG, GYRO_CONFIG_value);

		uint8_t ACCEL_FS_SEL = 1;
		ACCEL_CONFIG_value = 0 | ACCEL_FS_SEL << 3;
		write_reg(ACCEL_CONFIG, ACCEL_CONFIG_value);

		uint8_t accel_fchoice_b = 0;
		uint8_t A_DLPFCFG = 3;
		ACCEL_CONFIG_2_value = A_DLPFCFG | accel_fchoice_b << 3;
		write_reg(ACCEL_CONFIG_2, ACCEL_CONFIG_2_value);
		lsb_to_mg_accel = 4000.0f / 32768.0f;
		lsb_to_dps_gyro = 250.0f / 32768.0f;
    }
    //-----------end setting up gyro and accel-----
    {
		//enable i2c master mode
		uint8_t I2C_MST_EN = 1;
		USER_CTRL_value = 0|I2C_MST_EN<<5;
		write_reg(USER_CTRL,USER_CTRL_value);

		//set i2c bus clock 400kHz
		uint8_t I2C_MST_CLK = 13;
		I2C_MST_CTRL_value = I2C_MST_CLK;
		write_reg(I2C_MST_CTRL,I2C_MST_CTRL_value);
    }
    //checking magnetometrs value must be equal 72
    uint8_t who_am_i = read_reg_to_AK8963(0x00);
    printf("mag %02hx \r\n",who_am_i);
    //setting up magnetometrs
    {
    	CNTL1_value = 0x00;
    	write_register_to_AK8963(CNTL1,CNTL1_value);
    	delay(100);

    	write_register_to_AK8963(CNTL1,0x0f);
    	delay(100);

    	uint8_t buff[7];
    	read_registers_to_AK8963(ASAX,buff,3);
    	mag_gain_from_rom[0] = ((((float)buff[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
    	mag_gain_from_rom[1] = ((((float)buff[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
    	mag_gain_from_rom[2] = ((((float)buff[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;

    	CNTL1_value = 0x00;
    	write_register_to_AK8963(CNTL1,CNTL1_value);
    	delay(100);
    	CNTL1_value = 0x16;
    	write_register_to_AK8963(CNTL1,CNTL1_value);

    	read_registers_to_AK8963(HXL,buff,7);
    }

}

void Mpu9250::write_reg(uint8_t reg, uint8_t data) {
	mySPI.writeByte(device_accel_gyro, reg, data);
}

uint8_t Mpu9250::read_reg(uint8_t reg) {
	uint8_t rx;
	mySPI.readBytes(device_accel_gyro, reg, 1, &rx);
	return rx;
}

void Mpu9250::read_raw_data() {

	write_reg(I2C_SLV0_ADDR,AK8963_I2C_ADDR | 0x80); //Set the I2C slave addres of AK8963 and set for read.
	write_reg(I2C_SLV0_REG, HXL); //I2C slave 0 register address from where to begin data transfer
	write_reg(I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.


	read_registers(ACCEL_XOUT_H, rx_buffer, 21);
    raw_data_accel[0] = (rx_buffer[1]) | rx_buffer[0] << 8;
    raw_data_accel[1] = (rx_buffer[3]) | rx_buffer[2] << 8;
    raw_data_accel[2] = (rx_buffer[5]) | rx_buffer[4] << 8;

    raw_temp = (rx_buffer[7]) | rx_buffer[6] << 8;

    raw_data_gyro[0] = (rx_buffer[9]) | rx_buffer[8] << 8;
    raw_data_gyro[1] = (rx_buffer[11]) | rx_buffer[10] << 8;
    raw_data_gyro[2] = (rx_buffer[13]) | rx_buffer[12] << 8;

    raw_data_mag[0] = rx_buffer[14] | rx_buffer[15] << 8;
    raw_data_mag[1] = rx_buffer[16] | rx_buffer[17] << 8;
    raw_data_mag[2] = rx_buffer[18] | rx_buffer[19] << 8;


//    print_bytes(rx_buffer+14,7);

}

void Mpu9250::init_spi() {
    spi_bus_config_t config;
    config.mosi_io_num = 23;
    config.miso_io_num = 19;
    config.sclk_io_num = 18;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = 1000;
    config.flags = SPICOMMON_BUSFLAG_MASTER;
    config.intr_flags = 0;
    spi_bus_initialize(VSPI_HOST, &config, 0);
	ESP_ERROR_CHECK( mySPI.addDevice(0, 8000000, 5, &device_accel_gyro));

    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;
    dev_config.duty_cycle_pos = 128;  // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans = 0;  // 0 not used
    dev_config.cs_ena_posttrans = 0;  // 0 not used
    dev_config.clock_speed_hz = 8000000;
    dev_config.input_delay_ns = 0;
    dev_config.spics_io_num = 5;
    dev_config.flags = 0;  // 0 not used
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    ESP_ERROR_CHECK( spi_bus_add_device(VSPI_HOST, &dev_config, &device_accel_gyro));
}

void Mpu9250::delay(uint32_t delay_) {
    vTaskDelay(delay_ / portTICK_PERIOD_MS);
}

void Mpu9250::write_register_to_AK8963(uint8_t reg, uint8_t data) {
	write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR);

	write_reg(I2C_SLV0_REG, reg);

	write_reg(I2C_SLV0_DO, data);

	I2C_SLV0_CTRL_value = 0x80 | 1;
	write_reg(I2C_SLV0_CTRL, I2C_SLV0_CTRL_value);

//	read_register_to_AK8963(reg);
}

void Mpu9250::read_registers_to_AK8963(uint8_t reg, uint8_t* data, uint8_t cnt) {

	write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	write_reg(I2C_SLV0_REG , reg);

	I2C_SLV0_CTRL_value = 0x80 | cnt;
	write_reg(I2C_SLV0_CTRL , I2C_SLV0_CTRL_value);

	delay(5);
	read_registers(EXT_SENS_DATA_00,data,cnt);
}

uint8_t Mpu9250::read_reg_to_AK8963(uint8_t reg) {
	uint8_t data;
	read_registers(reg,&data,1);
	return data;
}

void Mpu9250::read_registers(uint8_t reg, uint8_t *data, uint8_t cnt) {
	spi_transaction_t transaction;
	transaction.flags = 0;
	transaction.cmd = 0;
	transaction.addr = reg | SPIBUS_READ;
	transaction.length = cnt * 8;
	transaction.rxlength = cnt * 8;
	transaction.user = NULL;
	transaction.tx_buffer = NULL;
	transaction.rx_buffer = data;
	ESP_ERROR_CHECK( spi_device_transmit(device_accel_gyro, &transaction));
}

void Mpu9250::read_data() {
	read_raw_data();

	accel       = Eigen::Vector3f(static_cast<float>(raw_data_accel[0]), static_cast<float>(raw_data_accel[1]), static_cast<float>(raw_data_accel[2]));
	anguar_velo = Eigen::Vector3f(static_cast<float>(raw_data_gyro[0]), static_cast<float>(raw_data_gyro[1]), static_cast<float>(raw_data_gyro[2]));
	mag         = Eigen::Vector3f(static_cast<float>(raw_data_mag[0]), static_cast<float>(raw_data_mag[1]), static_cast<float>(raw_data_mag[2]));

	accel       *= lsb_to_mg_accel;
	anguar_velo *= lsb_to_dps_gyro;
	mag         *= lsb_to_nT_mag;
}
