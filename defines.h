// ********* AlarmClock.h ****************
// Runs on TM4C123
// It defines the necessary interfaces to
// initialize, run and display clocks

#ifndef _DEFINES_H_
#define _DEFINES_H
#include <stdint.h>

#define LEDS (*((volatile uint32_t *)0x400253FC))
#define PI 3.14159265359
#define E4PI 31416L
typedef struct {
	float q0, q1, q2, q3;
} quaternion_t;

enum {NONE, SW1 = (1 << 0), SW2 = (1 << 1), SW3 = (1 << 2), SW4 = (1 << 3), SW5 = (1 << 4), SW6 = (1 << 5)};

void delay(uint32_t millsec);
void delay_micro(uint32_t microsec);
float invSqrt(float x);
	
// Utilities for quaternions
quaternion_t q_mul(quaternion_t, quaternion_t);
quaternion_t q_conjugate(quaternion_t Q);
float q_norm(quaternion_t Q);
#endif
