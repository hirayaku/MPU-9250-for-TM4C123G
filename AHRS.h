#ifndef _AHRS_h
#define _AHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
// Update quaterion w/ or w/o magnetic field measurement
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

// Get raw, pitch or roll
float CalcYaw(void);
float CalcPitch(void);
float CalcRoll(void);

// Get cosine of the angle between current axes and fixed z axis
float calc_cos_x(void);
float calc_cos_y(void);
float calc_cos_z(void);
#endif
