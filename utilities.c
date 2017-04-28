#include <stdint.h>
#include "defines.h"

// delay approximately millsec - works just fine if Bus is 80MHz
void delay(uint32_t millsec) {
	for(uint32_t i = 0; i < 10000 * millsec; i++);
}

void delay_micro(uint32_t microsec) {
	for(uint32_t i = 0; i < 10 * microsec; i++);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

quaternion_t q_mul(quaternion_t Q1, quaternion_t Q2) {
	quaternion_t result;
	result.q0 = Q1.q0 * Q2.q0 - Q1.q1 * Q2.q1 - Q1.q2 * Q2.q2 - Q1.q3 * Q2.q3;
	result.q1 = Q1.q0 * Q2.q1 + Q1.q1 * Q2.q0 + Q1.q2 * Q2.q3 - Q1.q3 * Q2.q2;
	result.q2 = Q1.q0 * Q2.q2 + Q1.q2 * Q2.q0 - Q1.q1 * Q2.q3 + Q1.q3 * Q2.q1;
	result.q3 = Q1.q0 * Q2.q3 + Q1.q3 * Q2.q0 + Q1.q1 * Q2.q2 - Q1.q2 * Q2.q1;
	return result;
}

quaternion_t q_conjugate(quaternion_t Q) {
	quaternion_t Q_bar = {Q.q0, -Q.q1, -Q.q2, -Q.q3};
	return Q_bar;
}

float q_norm(quaternion_t Q) {
	return invSqrt(Q.q0 * Q.q0 + Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3);
}
