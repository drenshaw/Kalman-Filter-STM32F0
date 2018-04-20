/******************** Kalman Filter Implementation **********************
Creator: Devin Renshaw
Description:
		Kalman filter for fusing data from accelerometer and gyroscope data
		Uses on-board gyroscope data or gyro and accelerometer data from
		MPU6050 with optional magnetometer to account for drift
*/

/* Includes */
#include "kalman.h"

/*
// Prediction

x = A*x + B*u

p = A*p*AT + Q

// Updates

Innovation = (H*p*HT + R)^-1

K = p*HT*Innovation
				OR
K = PH'(HPH_T + VRV_T)^-1

x = x + K*(y-H*x)

p = (I-K*H)*p

*/ 

/* Variable declarations and definitions */
// Variance of gyroscope (given by datasheet, generally)
const uint32_t sigma_gyro = 0;
const uint32_t sigma_g_sq = sigma_gyro * sigma_gyro;
// Variance of accelerometer (given by datasheet, generally)
const uint32_t sigma_accel = 0;
const uint32_t sigma_a_sq = sigma_accel * sigma_accel;
/* Arrays for storing update/prediction steps, and variables */
// x_k is state estimation
static uint32_t x_k[STATES][1];
// y_k is sensor measurements
static uint32_t y_k[SENSORS][1];
// F is the matrix to transform the old state to the new state
static uint32_t F[STATES][STATES];
static uint32_t H[SENSORS][STATES];
static uint32_t I[STATES][STATES] = {{1,0,0},{0,1,0},{0,0,1}};	// Identity matrix
static uint32_t W[STATES][STATES] = {{1,0,0},{0,1,0},{0,0,1}};
static uint32_t V[SENSORS][SENSORS]={{1,0},{0,1}};
static uint32_t R[SENSORS][SENSORS]={{sigma_g_sq,0},{0,sigma_a_sq}};
static uint32_t	Q[STATES][STATES];
static uint32_t	P[STATES][STATES];	// Set to Q before iteration

