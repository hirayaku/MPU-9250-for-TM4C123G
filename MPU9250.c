/* MPU9250.c
 * This file implements the functions defined in MPU9250.h
 * including: initialize MPU9250 to different SPI modes
 *						read data from MPU9250
 *						hibernate MPU9250
 *
 * This file is based on the MPU-9250 SPI library for Arduino by Brian Chen,
 * and ported to the TM4C123 platform by Tianhao Huang.
 * Modifications made to Brain's code:
 * 1) Rewrite read/write regs of MPU, since in TM4C123 
 *    we prefer to write SPI transfer functions by hand
 * 2) Add proper delays between two adjacent AK8963 register reads/writes, 
 *    since I2C is slow and AK8963 takes relative long to respond to the data changes
 * 3) Some minor errors in the calibration functions.
 *
 * Open source under MIT License.
 * Free to use or modify these codes. NO WARRANTY is guaranteed.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "./inc/tm4c123gh6pm.h"
#include "defines.h"
#include "MPU9250.h"
#include "MPU9250_reg.h"
#include "UART.h"

/* If you change SSI ports, please also modify these defines */
#define CS_n *((volatile uint32_t *)0x40007008)	// PD1 as CS_n
#define CS_H	0x02
#define CS_L	0x00
#define SPI_DATA SSI1_DR_R;											// SPI data register

#define DEBUG_PRINT

#ifdef DEBUG_PRINT_VERBOSE
# ifndef DEBUG_PRINT
# define DEBUG_PRINT
# endif
#endif

/* The sequence of data used to initialize MPU-9250
 * 1MHz SPI interface, +- 250DPS, +-2G
 * Accel data LPF 184Hz with 5.8ms delay; gyro data LPF 184Hz with 3ms delay; 
 * AK8963 read from I2C Master
 */
uint8_t MPU_init_data[16][2] = {
		{BITS_DLPF_CFG_188HZ, MPUREG_CONFIG},    // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz.
		{BITS_FS_1000DPS, MPUREG_GYRO_CONFIG},   // +-1000dps
		{BITS_FS_4G, MPUREG_ACCEL_CONFIG},       // +-4G
		{BITS_DLPF_CFG_188HZ, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
		{0x00, MPUREG_SMPLRT_DIV},						 	// Set sample rate to 1 kHz
		{0xCD, MPUREG_I2C_MST_CTRL},     				// I2C configuration multi-master  IIC 400KHz
		{0x30, MPUREG_USER_CTRL},        				// Disable FIFO, enable I2C Master mode (i.e. not pass-through mode) and set I2C_IF_DIS to disable slave mode I2C bus
		{0x10, MPUREG_INT_PIN_CFG},							// Be cautious to change this.
		{0xff, 0xff}														// End of the init data array
};

uint8_t MPU_reset_data[4][2] = {
	  {0x80, MPUREG_PWR_MGMT_1},
    {0x01, MPUREG_PWR_MGMT_1}, 
    {0x00, MPUREG_PWR_MGMT_2},
		{0xff, 0xff}
};
/* ----------------------- Internal API for reading/writing registers of MPU ----------------------------- */
// Send one byte via SPI and then wait for response
static uint8_t send_and_receive(uint8_t data) {
	while( (SSI1_SR_R & 0x00000001) == 0) {}		// wait until SSI transmit FIFO empty
	SSI1_DR_R = data;														// write into the data register, start data exchange
	while( (SSI1_SR_R & 0x00000004) == 0) {}		// wait until SSI receive FIFO not empty
	return SSI1_DR_R;														// ack received data
}

static inline uint8_t bytewrite_MPU(uint8_t write_addr, uint8_t write_data) {
	uint8_t return_data[2];
	CS_n = CS_L;																// Start transaction
	// delay_micro(100);
	return_data[0] = send_and_receive(write_addr);
	return_data[1] = send_and_receive(write_data);
	CS_n = CS_H;
	
	return return_data[1];
}

static inline BOOL byteread_MPU(uint8_t read_addr, uint8_t *read_data) {
	read_addr |= 0x80;
	*read_data = bytewrite_MPU(read_addr, 0);
	return TRUE;
}

static BOOL read_MPU(uint8_t read_addr, uint8_t *read_buffer, uint16_t bytes_count) {
	read_addr |= 0x80;
	CS_n = CS_L;
	send_and_receive(read_addr);
	for(uint16_t i = 0; i < bytes_count; i++) {
		read_buffer[i] = send_and_receive(0);
	}
	CS_n = CS_H;
	return TRUE;
}

// Data access from AK8963 normally takes 1ms to 2ms to finish
// Therefore don't use it too often
static void bytewrite_AK8963(uint8_t address, uint8_t data) {
	const uint16_t microsec = 500;
	bytewrite_MPU(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);			// Write the Slave 0 AK8963 address and the direction of the data acess
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_REG, address);							// AK8963 register address from where to begin data transfer
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_DO, data);									// The data to be written into AK8963
	delay_micro(microsec);
																														// Yes even if it's a write action, you have to set I2C_SLV0_EN
	bytewrite_MPU(MPUREG_I2C_SLV0_CTRL, 0x81);								// which datasheet says enables reading data from the slave.
	delay_micro(microsec);
}

static void byteread_AK8963(uint8_t address, uint8_t *buffer) {
	const uint16_t microsec = 500;
	bytewrite_MPU(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|0x80);
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_REG, address);
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_CTRL, 0x81);
	delay_micro(microsec);
	byteread_MPU(MPUREG_EXT_SENS_DATA_00, buffer);
	delay_micro(100);
}

// bytes_count should be less than 8.
static void read_AK8963(uint8_t address, uint8_t *buffer, uint8_t bytes_count) {
	const uint16_t microsec = 500;
	uint8_t CTRL_reg = 0x80 + bytes_count;
	bytewrite_MPU(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|0x80);
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_REG, address);
	delay_micro(microsec);
	bytewrite_MPU(MPUREG_I2C_SLV0_CTRL, CTRL_reg);
	delay_micro(microsec);
	read_MPU(MPUREG_EXT_SENS_DATA_00, buffer, bytes_count);
	delay_micro(100);
}

void read_raw_accel(int16_t *buffer) {
	uint8_t temp_bytes[6];
	read_MPU(MPUREG_ACCEL_XOUT_H, temp_bytes, 6);
	buffer[0] = ((int16_t)temp_bytes[0] << 8)|temp_bytes[1];
	buffer[1] = ((int16_t)temp_bytes[2] << 8)|temp_bytes[3];
	buffer[2] = ((int16_t)temp_bytes[4] << 8)|temp_bytes[5];
}

void read_raw_gyro(int16_t *buffer) {
	uint8_t temp_bytes[6];
	read_MPU(MPUREG_GYRO_XOUT_H, temp_bytes, 6);
	buffer[0] = ((int16_t)temp_bytes[0] << 8)|temp_bytes[1];
	buffer[1] = ((int16_t)temp_bytes[2] << 8)|temp_bytes[3];
	buffer[2] = ((int16_t)temp_bytes[4] << 8)|temp_bytes[5];
}

static inline void read_raw_mag(int16_t *buffer) {
	// Credit to Brain: if you do not read 7 bytes but only 6 bytes, the new measurement will not be latched
	// I didn't notice it in the datasheet!!
	uint8_t mag_rawdata[7]; 
	read_AK8963(AK8963_HXL, mag_rawdata, 7);
	buffer[0] = (int16_t)(mag_rawdata[0] | ((uint16_t)mag_rawdata[1] << 8));
	buffer[1] = (int16_t)(mag_rawdata[2] | ((uint16_t)mag_rawdata[3] << 8));
	buffer[2] = (int16_t)(mag_rawdata[4] | ((uint16_t)mag_rawdata[5] << 8));
}

/* --------------------- API for calibration and self-test at initialization -------------------- */
// Write an array of preset value with the form: {{addr, value}, {addr, value}, ...}
// If one sub-array equals {0xff, 0xff}, we consider it the end of the array.
// @input:									preset array
//													write_count 
//													If write_count equals 0, we write the whole preset array;
//													else we only write `write_count' pairs of value.
static void write_preset(uint8_t preset[][2], int write_count) {
	unsigned array_end = 0;
	while(preset[array_end][0] != 0xff || preset[array_end][1] != 0xff) {
		array_end++;
	}
	if(write_count == 0) {
		write_count = array_end;
	}
	else {
		if(array_end < write_count) {
			#ifdef DEBUG_PRINT
			UART_OutString("Incorrect parameter for write_preset. Write abort.\r\n");
			#endif
			return;
		}
	}
	for(unsigned i = 0; i < write_count; i++) {
		bytewrite_MPU(preset[i][1], preset[i][0]);
		delay_micro(400);
	}
}

// Take samples to get inherent offsets of sensor data
static void calibrate(MPU_param_t *MPU_param) {
		char msg[100] = {0};
    uint8_t data[12];												// data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    bytewrite_MPU(MPUREG_PWR_MGMT_1, 0x80);	// Write a one to bit 7 reset bit; toggle reset device
    delay(100);
		
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    bytewrite_MPU(MPUREG_PWR_MGMT_1, 0x01);  
    bytewrite_MPU(MPUREG_PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    bytewrite_MPU(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    bytewrite_MPU(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    bytewrite_MPU(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    bytewrite_MPU(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    bytewrite_MPU(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    bytewrite_MPU(MPUREG_CONFIG, 0x01);      								// Set low-pass filter to 188 Hz
    bytewrite_MPU(MPUREG_SMPLRT_DIV, 0x00);  								// Set sample rate to 1 kHz
    bytewrite_MPU(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);  		// Set gyro full-scale to 250 degrees per second, maximum sensitivity
    bytewrite_MPU(MPUREG_ACCEL_CONFIG, BITS_FS_2G); 				// Set accelerometer full-scale to 2 g, maximum sensitivity
		bytewrite_MPU(MPUREG_ACCEL_CONFIG_2, 0x01);							// Set accelerometer LPF to 184 Hz
    
    uint16_t  gyrosensitivity  = 131;   			// = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  			// = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    bytewrite_MPU(MPUREG_USER_CTRL, 0x40);   	// Enable FIFO
    bytewrite_MPU(MPUREG_FIFO_EN, 0x78);     	// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 10 samples in 10 milliseconds = 120 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    bytewrite_MPU(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    read_MPU(MPUREG_FIFO_COUNTH, data, 2); 		 // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;							 // How many sets of full gyro and accelerometer data for averaging
		sprintf(msg, "FIFO byte count: %d. Packet count: %d\r\n", fifo_count, packet_count);
		UART_OutString(msg);

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        read_MPU(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
				
				#ifdef DEBUG_PRINT_VERBOSE
				// Debugging output
				sprintf(msg, "## %d-th sample\r\n", ii);
				UART_OutString(msg);
				for(int i = 0; i < 3; i++) {
					sprintf(msg, "Accel[%d]: %d\r\n", i, accel_temp[i]);
					UART_OutString(msg);
				}
				for(int i = 0; i < 3; i++) {
					sprintf(msg, "Gyro[%d]: %d\r\n", i, gyro_temp[i]);
					UART_OutString(msg);
				}
				#endif
    }
    accel_bias[0] = ((accel_bias[0]) / packet_count + ACCEL_XOFF) / 2; // Normalize sums to get average count biases
    accel_bias[1] = ((accel_bias[1]) / packet_count + ACCEL_YOFF) / 2;
    accel_bias[2] = (accel_bias[2]) / packet_count;
    gyro_bias[0]  = (gyro_bias[0]) / packet_count + GYRO_XOFF;
    gyro_bias[1]  = (gyro_bias[1]) / packet_count + GYRO_YOFF;
    gyro_bias[2]  = (gyro_bias[2]) / packet_count + GYRO_ZOFF;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-(gyro_bias[0]+2)/4  >> 8) & 0x00FF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-(gyro_bias[0]+2)/4)       & 0x00FF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-(gyro_bias[1]+2)/4  >> 8) & 0x00FF;
    data[3] = (-(gyro_bias[1]+2)/4)       & 0x00FF;
    data[4] = (-(gyro_bias[2]+2)/4  >> 8) & 0x00FF;
    data[5] = (-(gyro_bias[2]+2)/4)       & 0x00FF;

    // Push gyro biases to hardware registers
		// The offsets stored in these registers will be calibrate newly sampled values
    bytewrite_MPU(MPUREG_XG_OFFS_USRH, data[0]);
    bytewrite_MPU(MPUREG_XG_OFFS_USRL, data[1]);
    bytewrite_MPU(MPUREG_YG_OFFS_USRH, data[2]);
    bytewrite_MPU(MPUREG_YG_OFFS_USRL, data[3]);
    bytewrite_MPU(MPUREG_ZG_OFFS_USRH, data[4]);
    bytewrite_MPU(MPUREG_ZG_OFFS_USRL, data[5]);
  
    // Output scaled gyro biases for display in the main program
    MPU_param->gyro_bias[0] = (int16_t)gyro_bias[0];
    MPU_param->gyro_bias[1] = (int16_t)gyro_bias[1];
    MPU_param->gyro_bias[2] = (int16_t)gyro_bias[2];
		MPU_param->gyro_bias_div = gyrosensitivity;

		// Output scaled accelerometer biases for display in the main program
    MPU_param->accel_bias[0] = (int16_t)accel_bias[0];
    MPU_param->accel_bias[1] = (int16_t)accel_bias[1];
    MPU_param->accel_bias[2] = (int16_t)accel_bias[2];
		MPU_param->accel_bias_div = accelsensitivity;
	
		#ifdef DEBUG_PRINT
		// Debugging output
		sprintf(msg, "accel_bias in +-2g scale:\r\n");
		UART_OutString(msg);
		for(int i = 0; i < 3; i++) {
			sprintf(msg, "accel_bias[%d]: %f (%d)\r\n", i, MPU_param->accel_bias[i]/(float)MPU_param->accel_bias_div, MPU_param->accel_bias[i]);
			UART_OutString(msg);
		}
		sprintf(msg, "gyro_bias in +-250dps scale:\r\n");
		UART_OutString(msg);
		for(int i = 0; i < 3; i++) {
			sprintf(msg, "gyro_bias[%d]: %f (%d)\r\n", i, MPU_param->gyro_bias[i]/(float)MPU_param->accel_bias_div, MPU_param->gyro_bias[i]);
			UART_OutString(msg);
		}
		#endif
}

static void fast_calibrate(MPU_param_t *MPU_param) {
		char msg[100] = {0};
    uint8_t data[12];												// data array to hold accelerometer and gyro x, y, z, data
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    bytewrite_MPU(MPUREG_PWR_MGMT_1, 0x80);	// Write a one to bit 7 reset bit; toggle reset device
    delay(100);
		
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    bytewrite_MPU(MPUREG_PWR_MGMT_1, 0x01);  
    bytewrite_MPU(MPUREG_PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    bytewrite_MPU(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    bytewrite_MPU(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    bytewrite_MPU(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    bytewrite_MPU(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    bytewrite_MPU(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    bytewrite_MPU(MPUREG_CONFIG, 0x01);      								// Set low-pass filter to 188 Hz
    bytewrite_MPU(MPUREG_SMPLRT_DIV, 0x00);  								// Set sample rate to 1 kHz
		bytewrite_MPU(MPUREG_ACCEL_CONFIG_2, 0x01);							// Set accelerometer LPF to 184 Hz
    
    uint16_t  gyrosensitivity  = 131;   			// = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  			// = 16384 LSB/g

    accel_bias[0] = ACCEL_XOFF;
    accel_bias[1] = ACCEL_YOFF;
    accel_bias[2] = ACCEL_ZOFF;
    gyro_bias[0]  = GYRO_XOFF;
    gyro_bias[1]  = GYRO_YOFF;
    gyro_bias[2]  = GYRO_ZOFF;
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-(gyro_bias[0]+2)/4  >> 8) & 0x00FF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-(gyro_bias[0]+2)/4)       & 0x00FF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-(gyro_bias[1]+2)/4  >> 8) & 0x00FF;
    data[3] = (-(gyro_bias[1]+2)/4)       & 0x00FF;
    data[4] = (-(gyro_bias[2]+2)/4  >> 8) & 0x00FF;
    data[5] = (-(gyro_bias[2]+2)/4)       & 0x00FF;

    // Push gyro biases to hardware registers
		// The offsets stored in these registers will be calibrate newly sampled values
    bytewrite_MPU(MPUREG_XG_OFFS_USRH, data[0]);
    bytewrite_MPU(MPUREG_XG_OFFS_USRL, data[1]);
    bytewrite_MPU(MPUREG_YG_OFFS_USRH, data[2]);
    bytewrite_MPU(MPUREG_YG_OFFS_USRL, data[3]);
    bytewrite_MPU(MPUREG_ZG_OFFS_USRH, data[4]);
    bytewrite_MPU(MPUREG_ZG_OFFS_USRL, data[5]);
  
    // Output scaled gyro biases for display in the main program
    MPU_param->gyro_bias[0] = (int16_t)gyro_bias[0];
    MPU_param->gyro_bias[1] = (int16_t)gyro_bias[1];
    MPU_param->gyro_bias[2] = (int16_t)gyro_bias[2];
		MPU_param->gyro_bias_div = gyrosensitivity;

		// Output scaled accelerometer biases for display in the main program
    MPU_param->accel_bias[0] = (int16_t)accel_bias[0];
    MPU_param->accel_bias[1] = (int16_t)accel_bias[1];
    MPU_param->accel_bias[2] = (int16_t)accel_bias[2];
		MPU_param->accel_bias_div = accelsensitivity;
}

static void calibrate_mag(MPU_param_t *MPU_param){
    uint8_t response[3];
    float data;
    int i;
		
		//write_preset(MPU_reset_data, 0);
		write_preset(MPU_init_data, 0);
	  bytewrite_AK8963(AK8963_CNTL2, 0x01);																// reset AK8963
	  delay(10);
    bytewrite_AK8963(AK8963_CNTL1, BITS_AK8963_POWERDOWN);              // set AK8963 to Power Down
    delay(10);                                                					// long wait between AK8963 mode changes
    bytewrite_AK8963(AK8963_CNTL1, BITS_AK8963_ROM);      							// set AK8963 to FUSE ROM access
    delay(10);                                                          // long wait between AK8963 mode changes
		byteread_AK8963(AK8963_ASAX, response);															// read sensitivity adjustment values
		byteread_AK8963(AK8963_ASAY, response+1);
		byteread_AK8963(AK8963_ASAZ, response+2);

		#ifdef DEBUG_PRINT
		char msg[40];
		UART_OutString("ASAX data from AK8963:\r\n");
		for(i = 0; i < 3; i++) {
			sprintf(msg, "%5d	", (uint8_t)response[i]);
			UART_OutString(msg);
		}
		UART_OutString("\r\n");
		#endif
		
    for(i = 0; i < 3; i++) {
        data=response[i];
        MPU_param->mag_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
    bytewrite_AK8963(AK8963_CNTL1, BITS_AK8963_POWERDOWN); // set AK8963 to Power Down
    delay(10);
}

static void fast_calibrate_mag(MPU_param_t *MPU_param) {
	write_preset(MPU_init_data, 0);
	uint16_t response[3] = {ASAX, ASAY, ASAZ};
	for(int i = 0; i < 3; i++) {
			MPU_param->mag_ASA[i] = ((response[i]-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
	}
}

static void preconfig(MPU_param_t *MPU_param, BOOL fast_config) {
	
	if(fast_config == TRUE) {
		fast_calibrate(MPU_param);
		fast_calibrate_mag(MPU_param);
	}
	else {
		calibrate(MPU_param);
		calibrate_mag(MPU_param);
	}
	
	bytewrite_AK8963(AK8963_CNTL2, 0x01);
	bytewrite_AK8963(AK8963_CNTL1, BITS_AK8963_100HZ|AK8963_16BITS);
	
	char msg[40];
	uint8_t buffer;

	if(!whoami_MPU()) {
		UART_OutString("Wrong MPU device ID.\r\n");
		LEDS = 0x02;
		exit(EXIT_FAILURE);
	}
	if(!whoami_AK8963()) {
		UART_OutString("Wrong AK8963 device ID.\r\n");
		LEDS = 0x02;
		exit(EXIT_FAILURE);
	}
	
	byteread_AK8963(AK8963_CNTL1, &buffer);
	sprintf(msg, "AK8963 CNTL1 reg: 0x%02x\r\n", buffer);
	UART_OutString(msg);
	
	#ifdef DEBUG_PRINT_VERBOSE
	int16_t mag_data[3], accel_data[3], gyro_data[3];
	unsigned count = 0;
	while(count < 100) {
		count++;
		delay(10);
		read_raw_accel(accel_data);
		read_raw_gyro(gyro_data);
		read_raw_mag(mag_data);
		sprintf(msg, "%5d	%5d	%5d	", accel_data[0], accel_data[1], accel_data[2]);
		UART_OutString(msg);
		sprintf(msg, "%5d	%5d	%5d	", gyro_data[0], gyro_data[1], gyro_data[2]);
		UART_OutString(msg);
		sprintf(msg, "%5d	%5d	%5d	", mag_data[0], mag_data[1], mag_data[2]);
		UART_OutString(msg);
		UART_OutString("\r\n");
	}
	#endif
}

// Initialize MPU with the given mode, and set up SPI interface
MPU_param_t init_MPU(MODE_t mode) {
	uint32_t freq_div = ((mode == SLOW)?80:4);
	// First init SPI interface on PD0-PD3
	SYSCTL_RCGCSSI_R |= 0x02;
	SYSCTL_RCGCGPIO_R |= 0x08;								// activate port D and then wait for bus
	while((SYSCTL_PRGPIO_R & 0x08) == 0) {};
																						// configure as SSI ports
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0xFFFF00F0) + 0x00002202;
	GPIO_PORTD_AMSEL_R = 0;										// disable analog output
	GPIO_PORTD_AFSEL_R |= 0x0D;								// enable alt function on PD0, 2, 3
	GPIO_PORTD_DEN_R |= 0x0F;									// enable digital ports
	GPIO_PORTD_DIR_R |= 0x02;									// Make PD1 (CS_n) output
	CS_n = CS_H;															// CS_n asserted
	
	SSI1_CR1_R = 0x0;													// disable SSI
	SSI1_CPSR_R = freq_div;										// make SSI bandwidth low 1MHz or high 20MHz
	SSI1_CR0_R = 0x000000C7;									// SPO =1, SPH=1; freescale; 8 bits of data at one time
	SSI1_CR1_R |= 0x00000002;									// enable SSI
	
	MPU_param_t MPU_param;
	preconfig(&MPU_param, TRUE);							// Fast calibrate using preset values, and write preset values to MPU-9250 registers
	set_accel_scale_MPU(&MPU_param, BITS_FS_4G);
	set_gyro_scale_MPU(&MPU_param, BITS_FS_1000DPS);
	
	//whoami_AK8963();
	UART_OutString("Initilization ends\r\n");
	return MPU_param;
}

uint32_t set_accel_scale_MPU(MPU_param_t *MPU_param, uint32_t scale){
    uint8_t temp_scale;
		byteread_MPU(MPUREG_ACCEL_CONFIG, &temp_scale);
    bytewrite_MPU(MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            MPU_param->accel_scale=1;
        break;
        case BITS_FS_4G:
            MPU_param->accel_scale=2;
        break;
        case BITS_FS_8G:
            MPU_param->accel_scale=4;
        break;
        case BITS_FS_16G:
            MPU_param->accel_scale=8;
        break;   
    }
		
    #ifdef DEBUG_PRINT_VERBOSE
		char msg[100];
		// 0x00 - 2G; 0x08 - 4G; 0x10 - 8G; 0x18 - 16G
		sprintf(msg, "Previous accel config: 0x%x. Current config: 0x%x.\r\n", temp_scale, scale);
		UART_OutString(msg);
		#endif

    return temp_scale;
}

uint32_t set_gyro_scale_MPU(MPU_param_t *MPU_param, uint32_t scale){
    uint8_t temp_scale;
		byteread_MPU(MPUREG_GYRO_CONFIG, &temp_scale);
    bytewrite_MPU(MPUREG_GYRO_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_250DPS:
            MPU_param->gyro_scale=1;
        break;
        case BITS_FS_500DPS:
            MPU_param->gyro_scale=2;
        break;
        case BITS_FS_1000DPS:
            MPU_param->gyro_scale=4;
        break;
        case BITS_FS_2000DPS:
            MPU_param->gyro_scale=8;
        break;   
    }
		
    #ifdef DEBUG_PRINT_VERBOSE
		char msg[100];
		// 0x00 - 250DPS; 0x08 - 500DPS; 0x10 - 1000DPS; 0x18 - 2000DPS
		sprintf(msg, "Previous gyro config: 0x%x. Current config: 0x%x.\r\n", temp_scale, scale);
		UART_OutString(msg);
		#endif
		
    return temp_scale;
}

/*                                 WHO AM I?
 * usage: call these functions to know if SPI/I2C-Master is working correctly. 
 * It checks the I2C address of the mpu9250 which should be 0x71, 
 * and I2C address of the AK8963 which should be 0x48.
 */
BOOL whoami_MPU(){
	uint8_t response;
	byteread_MPU(MPUREG_WHOAMI, &response);
	
	#ifdef DEBUG_PRINT
	char msg[40];
	sprintf(msg, "WHO AM I MPU: %s\r\n", (response == MPU_DEVICE_ID)?"Passed":"Failed");
	UART_OutString(msg);
	#endif
	
	return (BOOL)(response == MPU_DEVICE_ID);
}

BOOL whoami_AK8963(){
	uint8_t buffer;
	byteread_AK8963(AK8963_WIA, &buffer);
	#ifdef DEBUG_PRINT
	char msg[40];
	sprintf(msg, "WHO AM I AK8963: %s\r\n", (buffer == AK8963_DEVICE_ID)?"Passed":"Failed");
	UART_OutString(msg);
	#endif
	return (BOOL)(buffer == AK8963_DEVICE_ID);
}

// Cause MPU to sleep
void hibernate_MPU(void);

// Request raw data from MPU
void read_accel_gyro(const MPU_param_t *MPU_param, MPU_data_t *buffer) {
	int16_t rawdata[6];
	read_raw_accel(rawdata);
	read_raw_gyro(rawdata+3);
	for(unsigned i = 0; i < 3; i++) {
		buffer->accel[i] = (rawdata[i]*MPU_param->accel_scale - MPU_param->accel_bias[i])/(float)MPU_param->accel_bias_div;
		buffer->gyro[i] = ((int32_t)rawdata[i+3]*MPU_param->gyro_scale * E4PI)/(float)(MPU_param->gyro_bias_div*10000*180);
	}
}

void read_mag(const MPU_param_t *MPU_param, MPU_data_t *buffer) {
	int16_t rawdata[3];
	read_raw_mag(rawdata);
	for(unsigned i = 0; i < 3; i++) {
		buffer->magn[i] = (float)rawdata[i]*MPU_param->mag_ASA[i];
	}
}

void read_all(const MPU_param_t *MPU_param, MPU_data_t *buffer) {
	read_accel_gyro(MPU_param, buffer);
	read_mag(MPU_param, buffer);
}
