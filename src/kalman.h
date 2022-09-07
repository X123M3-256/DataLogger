#ifndef KALMAN_INCLUDED
#define KALMAN_INCLUDED
#include "vectormath.h"

typedef struct
{
vector3_t position;
vector3_t velocity;
quaternion_t orientation;
vector3_t accel_bias;
vector3_t gyro_bias;
}state_t;

void kalman_predict(state_t* state,double* covariance,double* input);
void kalman_update(state_t* state,double* covariance,double* measurement);

#define POSITION_NOISE 1.0
#define VELOCITY_NOISE 0.1
#define GYRO_NOISE 0.01
#define ACCEL_NOISE 0.0001
#define MAG_NOISE 0.0001
#define MAG_DIRECTION vector3(0.342,0,0.9397)

//Add additional uncertainty to account for integration error
#define INTEGRATION_NOISE  0.005


#endif

