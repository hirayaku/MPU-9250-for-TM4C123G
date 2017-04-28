// ******************* MPU99250.h *************************
// Runs on TMC4C123
// TMC4C123 communicates with MPU-9250 using SPI interface
// This file defines functions to activate and acquire data
// from MPU-9250

#include <stdint.h>

#define ACCEL_XOFF 630
#define ACCEL_YOFF 145
#define ACCEL_ZOFF 740
#define GYRO_XOFF -203
#define GYRO_YOFF 89
#define GYRO_ZOFF -220
#define ASAX 176
#define ASAY 176
#define ASAZ 165
typedef enum {FALSE, TRUE} BOOL;

// MPU-9250 configuration struct
typedef struct {
	int16_t accel_bias[3], gyro_bias[3];
	float accel_bias_div, gyro_bias_div;
	int16_t accel_scale, gyro_scale;
	float mag_ASA[3];
	int delta;					// sampling interval
} MPU_param_t;

// MPU_data_t struct describes the raw data acquired from MPU-9250
typedef struct {
	// x, y, z axis
	float accel[3];
	float gyro[3];
	float magn[3];
} MPU_data_t;

// MPU-9250 has different modes which has different sampling rates.
typedef enum {SLEEP, SLOW, FAST} MODE_t;

// Initialize MPU with the given mode,
// set up SPI interface
MPU_param_t init_MPU(MODE_t mode);

/*								Set accelerometer and gyroscope scales
 * set new scale and return previous scale byte
 * Input:		uint32_t									See the macro defined in MPU9250_reg.h, such as BITS_FS_250DPS.
 * Output:	uint32_t									Same as above
 */
uint32_t set_accel_scale_MPU(MPU_param_t *MPU_param, uint32_t scale);
uint32_t set_gyro_scale_MPU(MPU_param_t *MPU_param, uint32_t scale);

/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 and AK8963 which should be 0x71
 */
BOOL whoami_MPU(void);
BOOL whoami_AK8963(void);

// Cause MPU to sleep
void hibernate_MPU(void);

void read_raw_accel(int16_t *buffer);
void read_raw_gyro(int16_t *buffer);

// Request data from MPU and return current data
void read_accel_gyro(const MPU_param_t *MPU_param, MPU_data_t *buffer);
void read_mag(const MPU_param_t *MPU_param, MPU_data_t *buffer);
void read_all(const MPU_param_t *MPU_param, MPU_data_t *buffer);
