#include<cstring>
#include<arm_math.h>
#include "kalman.h"





#define NUM_STATES 15
#define NUM_INPUTS 6
#define NUM_MEASUREMENTS 9

#define DT 0.01


//Computes A+B
void kmatrix_add(int rows,int cols,const double* a,const double* b,double* out) 
{
	for(int row=0;row<rows;row++)
	for(int col=0;col<cols;col++)
	{
	out[col+row*cols]=a[col+row*cols]+b[col+row*cols];
	}
}

//Computes A-B
void kmatrix_sub(int rows,int cols,const double* a,const double* b,double* out) 
{
	for(int row=0;row<rows;row++)
	for(int col=0;col<cols;col++)
	{
	out[col+row*cols]=a[col+row*cols]-b[col+row*cols];
	}
}

//Computes AB
void kmatrix_mult(int a_row,int a_col,int b_row,int b_col,const double* a,const double* b,double* out) 
{
	for(int row=0;row<a_row;row++)
	for(int col=0;col<b_col;col++)
	{
	out[col+row*b_col]=0;
		for(int i=0;i<a_col;i++)
		{
		out[col+row*b_col]+=a[i+row*a_col]*b[col+i*b_col];
		}
	}
}

//Computes AB^T
void kmatrix_mult_transpose(int a_row,int a_col,int b_row,int b_col,const double* a,const double* b,double* out) 
{
	for(int row=0;row<a_row;row++)
	for(int col=0;col<b_row;col++)
	{
	out[col+row*b_row]=0;
		for(int i=0;i<a_col;i++)
		{
		out[col+row*b_row]+=a[i+row*a_col]*b[i+col*b_col];
		}
	}
}

void kmatrix_inverse(int a_row,double* a,double* out)
{
arm_matrix_instance_f64 src={(uint16_t)a_row,(uint16_t)a_row,a};
arm_matrix_instance_f64 dst={(uint16_t)a_row,(uint16_t)a_row,out};
arm_mat_inverse_f64(&src,&dst);
}


void predict(state_t* reference,double* input,double* predicted_state)
{
vector3_t accel=vector3_sub(vector3(0,0,9.81),quaternion_vector(reference->orientation,vector3(input[0],input[1],input[2])));

//Position
predicted_state[0]=reference->velocity.x*DT+0.5*accel.x*DT*DT;//Latitude
predicted_state[1]=reference->velocity.y*DT+0.5*accel.y*DT*DT;//Longitude
predicted_state[2]=reference->velocity.z*DT+0.5*accel.z*DT*DT;//Altitude
//Velocity
predicted_state[3]=reference->velocity.x+accel.x*DT;//North
predicted_state[4]=reference->velocity.y+accel.y*DT;//East
predicted_state[5]=reference->velocity.z+accel.z*DT;//Down
//Rotation
predicted_state[6]=input[3]*DT;
predicted_state[7]=input[4]*DT;
predicted_state[8]=input[5]*DT;
}

void compute_state_transition_jacobian(state_t* reference,double* input,double* mat)
{
memset(mat,0,NUM_STATES*NUM_STATES*sizeof(double));

vector3_t a_x=quaternion_vector(reference->orientation,vector3(0,-input[2],input[1]));
vector3_t a_y=quaternion_vector(reference->orientation,vector3(input[2],0,-input[0]));
vector3_t a_z=quaternion_vector(reference->orientation,vector3(-input[1],input[0],0));

//Position
mat[0*NUM_STATES+0]=1.0;
mat[1*NUM_STATES+1]=1.0;
mat[2*NUM_STATES+2]=1.0;
mat[0*NUM_STATES+3]=DT;
mat[1*NUM_STATES+4]=DT;
mat[2*NUM_STATES+5]=DT;
mat[0*NUM_STATES+6]=0.5*a_x.x*DT*DT;
mat[0*NUM_STATES+7]=0.5*a_y.x*DT*DT;
mat[0*NUM_STATES+8]=0.5*a_z.x*DT*DT;
mat[1*NUM_STATES+6]=0.5*a_x.y*DT*DT;
mat[1*NUM_STATES+7]=0.5*a_y.y*DT*DT;
mat[1*NUM_STATES+8]=0.5*a_z.y*DT*DT;
mat[2*NUM_STATES+6]=0.5*a_x.z*DT*DT;
mat[2*NUM_STATES+7]=0.5*a_y.z*DT*DT;
mat[2*NUM_STATES+8]=0.5*a_z.z*DT*DT;

//Velocity
mat[3*NUM_STATES+3]=1.0;
mat[4*NUM_STATES+4]=1.0;
mat[5*NUM_STATES+5]=1.0;
mat[3*NUM_STATES+6]=a_x.x*DT;
mat[3*NUM_STATES+7]=a_y.x*DT;
mat[3*NUM_STATES+8]=a_z.x*DT;
mat[4*NUM_STATES+6]=a_x.y*DT;
mat[4*NUM_STATES+7]=a_y.y*DT;
mat[4*NUM_STATES+8]=a_z.y*DT;
mat[5*NUM_STATES+6]=a_x.z*DT;
mat[5*NUM_STATES+7]=a_y.z*DT;
mat[5*NUM_STATES+8]=a_z.z*DT;

//Rotation
mat[6*NUM_STATES+6]=1.0;
mat[7*NUM_STATES+7]=1.0;
mat[8*NUM_STATES+8]=1.0;
}

#define SQR(x) ((x)*(x))

void compute_process_noise(state_t* reference,double* mat)
{
memset(mat,0,NUM_STATES*NUM_STATES*sizeof(double));

mat[0*NUM_STATES+0]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT);
mat[1*NUM_STATES+1]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT);
mat[2*NUM_STATES+2]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT);

mat[0*NUM_STATES+3]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);
mat[1*NUM_STATES+4]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);
mat[2*NUM_STATES+5]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);

mat[3*NUM_STATES+0]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);
mat[4*NUM_STATES+1]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);
mat[5*NUM_STATES+2]=SQR(0.5*(ACCEL_NOISE+INTEGRATION_NOISE)*DT*DT*DT);


mat[3*NUM_STATES+3]=SQR((ACCEL_NOISE+INTEGRATION_NOISE)*DT);
mat[4*NUM_STATES+4]=SQR((ACCEL_NOISE+INTEGRATION_NOISE)*DT);
mat[5*NUM_STATES+5]=SQR((ACCEL_NOISE+INTEGRATION_NOISE)*DT);

mat[6*NUM_STATES+6]=SQR((GYRO_NOISE+INTEGRATION_NOISE)*DT);
mat[7*NUM_STATES+7]=SQR((GYRO_NOISE+INTEGRATION_NOISE)*DT);
mat[8*NUM_STATES+8]=SQR((GYRO_NOISE+INTEGRATION_NOISE)*DT);
}


void compute_measurement(state_t* reference,double* state,double* measurement)
{
quaternion_t orientation=quaternion_mult(quaternion_axis(*((vector3_t*)(state+6))),reference->orientation);
quaternion_t orientation_inverse=quaternion_inverse(orientation);
*((vector3_t*)(measurement+0))=*((vector3_t*)(state+0));
*((vector3_t*)(measurement+3))=*((vector3_t*)(state+3));
*((vector3_t*)(measurement+6))=quaternion_vector(orientation_inverse,MAG_DIRECTION);
}

void compute_measurement_jacobian(state_t* state,double* mat)
{
quaternion_t orientation_inverse=quaternion_inverse(state->orientation);
//printf("%f %f %f %f\n",orientation_inverse.w,orientation_inverse.i,orientation_inverse.j,orientation_inverse.k);
vector3_t m=quaternion_vector(orientation_inverse,MAG_DIRECTION);

memset(mat,0,NUM_MEASUREMENTS*NUM_STATES*sizeof(double));

//Position
mat[0*NUM_STATES+0]=1.0;
mat[1*NUM_STATES+1]=1.0;
mat[2*NUM_STATES+2]=1.0;

//Velocity
mat[3*NUM_STATES+3]=1.0;
mat[4*NUM_STATES+4]=1.0;
mat[5*NUM_STATES+5]=1.0;

//Magnetometer
mat[6*NUM_STATES+6]=0.0;
mat[7*NUM_STATES+6]=-m.z;
mat[8*NUM_STATES+6]=m.y;

mat[6*NUM_STATES+7]=m.z;
mat[7*NUM_STATES+7]=0.0;
mat[8*NUM_STATES+7]=-m.x;

mat[6*NUM_STATES+8]=-m.y;
mat[7*NUM_STATES+8]=m.x;
mat[8*NUM_STATES+8]=0.0;
}

void compute_measurement_noise(state_t* state,double* mat)
{
memset(mat,0,NUM_MEASUREMENTS*NUM_MEASUREMENTS*sizeof(double));

//Position
mat[0*NUM_MEASUREMENTS+0]=POSITION_NOISE*POSITION_NOISE;
mat[1*NUM_MEASUREMENTS+1]=POSITION_NOISE*POSITION_NOISE;
mat[2*NUM_MEASUREMENTS+2]=POSITION_NOISE*POSITION_NOISE;

//Velocity
mat[3*NUM_MEASUREMENTS+3]=VELOCITY_NOISE*VELOCITY_NOISE;
mat[4*NUM_MEASUREMENTS+4]=VELOCITY_NOISE*VELOCITY_NOISE;
mat[5*NUM_MEASUREMENTS+5]=VELOCITY_NOISE*VELOCITY_NOISE;

//Magnetometer
mat[6*NUM_MEASUREMENTS+6]=MAG_NOISE*MAG_NOISE;
mat[7*NUM_MEASUREMENTS+7]=MAG_NOISE*MAG_NOISE;
mat[8*NUM_MEASUREMENTS+8]=MAG_NOISE*MAG_NOISE;
}


void kalman_predict(state_t* state,double* covariance,double* input)
{
//Compute matrices
double state_transition_jacobian[NUM_STATES*NUM_STATES];
double process_noise[NUM_STATES*NUM_STATES];

compute_state_transition_jacobian(state,input,state_transition_jacobian);

//printf("\nState Transition Jacobian\n");
//dump_matrix(NUM_STATES,NUM_STATES,state_transition_jacobian);


compute_process_noise(state,process_noise);

//Predict state
double predicted_state[NUM_STATES];
predict(state,input,predicted_state);



//Predict covariance
double temp1[NUM_STATES*NUM_STATES];
double temp2[NUM_STATES*NUM_STATES];
kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,covariance,state_transition_jacobian,temp1);
kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,state_transition_jacobian,temp1,temp2);
kmatrix_add(NUM_STATES,NUM_STATES,temp2,process_noise,covariance);

//Update state
double k1=111132.09-566.05*cos(2*state->position.x)+1.2*cos(4*state->position.x);
double k2=111415.13*cos(state->position.x)-94.55*cos(3*state->position.x)+0.12*cos(5*state->position.x);
state->position.x+=predicted_state[0]/k1;
state->position.y+=predicted_state[1]/k2;
state->position.z+=predicted_state[2];
state->velocity=*((vector3_t*)(predicted_state+3));
state->orientation=quaternion_mult(quaternion_axis(*((vector3_t*)(predicted_state+6))),state->orientation);
//printf("\nCovariance\n");
//dump_matrix(NUM_STATES,NUM_STATES,covariance);
}

void kalman_update(state_t* state,double* covariance,double* measurement)
{
//Compute local measurement
double k1=111132.09-566.05*cos(2*state->position.x)+1.2*cos(4*state->position.x);
double k2=111415.13*cos(state->position.x)-94.55*cos(3*state->position.x)+0.12*cos(5*state->position.x);
measurement[0]=(measurement[0]-state->position.x)*k1;
measurement[1]=(measurement[1]-state->position.y)*k2;
measurement[2]=measurement[2]-state->position.z;



double measurement_jacobian[NUM_MEASUREMENTS*NUM_STATES];
double measurement_noise[NUM_MEASUREMENTS*NUM_MEASUREMENTS];

compute_measurement_jacobian(state,measurement_jacobian);
compute_measurement_noise(state,measurement_noise);

//printf("\nCovariance\n");
//dump_matrix(NUM_STATES,NUM_STATES,covariance);

double predicted_state[NUM_STATES]={0,0,0,state->velocity.x,state->velocity.y,state->velocity.z,0,0,0};

//Calculate innovation measurement
double predicted_measurement[NUM_MEASUREMENTS];
compute_measurement(state,predicted_state,predicted_measurement);
double measurement_residual[NUM_MEASUREMENTS];
kmatrix_sub(NUM_MEASUREMENTS,1,measurement,predicted_measurement,measurement_residual);

//Compute innovation covariance
double temp3[NUM_STATES*NUM_MEASUREMENTS];
kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_MEASUREMENTS,NUM_STATES,covariance,measurement_jacobian,temp3);
double temp4[NUM_MEASUREMENTS*NUM_MEASUREMENTS];
kmatrix_mult(NUM_MEASUREMENTS,NUM_STATES,NUM_STATES,NUM_MEASUREMENTS,measurement_jacobian,temp3,temp4);
double covariance_residual[NUM_MEASUREMENTS*NUM_MEASUREMENTS];
kmatrix_add(NUM_MEASUREMENTS,NUM_MEASUREMENTS,temp4,measurement_noise,covariance_residual);

//Compute Kalman gain
double covariance_residual_inverse[NUM_MEASUREMENTS*NUM_MEASUREMENTS];
kmatrix_inverse(NUM_MEASUREMENTS,covariance_residual,covariance_residual_inverse);

double kalman_gain[NUM_STATES*NUM_MEASUREMENTS];
kmatrix_mult(NUM_STATES,NUM_MEASUREMENTS,NUM_MEASUREMENTS,NUM_MEASUREMENTS,temp3,covariance_residual_inverse,kalman_gain);

//printf("\nKalman gain\n");
//dump_matrix(NUM_STATES,NUM_MEASUREMENTS,kalman_gain);
 
//Compute corrected state
double temp6[NUM_STATES];
double corrected_state[NUM_STATES];
kmatrix_mult(NUM_STATES,NUM_MEASUREMENTS,NUM_MEASUREMENTS,1,kalman_gain,measurement_residual,temp6);
kmatrix_add(NUM_STATES,1,predicted_state,temp6,corrected_state);

//Compute corrected covariance
double temp1[NUM_STATES*NUM_STATES];
double temp2[NUM_STATES*NUM_STATES];
const double identity[NUM_STATES*NUM_STATES]={
1,0,0,0,0,0,0,0,0,
0,1,0,0,0,0,0,0,0,
0,0,1,0,0,0,0,0,0,
0,0,0,1,0,0,0,0,0,
0,0,0,0,1,0,0,0,0,
0,0,0,0,0,1,0,0,0,
0,0,0,0,0,0,1,0,0,
0,0,0,0,0,0,0,1,0,
0,0,0,0,0,0,0,0,1,
};
kmatrix_mult(NUM_STATES,NUM_MEASUREMENTS,NUM_MEASUREMENTS,NUM_STATES,kalman_gain,measurement_jacobian,temp2);
kmatrix_sub(NUM_STATES,NUM_STATES,identity,temp2,temp1);
double temp7[NUM_STATES*NUM_STATES];
kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,covariance,temp1,temp2);
kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,temp1,temp2,temp7);
kmatrix_mult_transpose(NUM_MEASUREMENTS,NUM_MEASUREMENTS,NUM_STATES,NUM_MEASUREMENTS,measurement_noise,kalman_gain,temp3);
kmatrix_mult(NUM_STATES,NUM_MEASUREMENTS,NUM_MEASUREMENTS,NUM_STATES,kalman_gain,temp3,temp1);
kmatrix_add(NUM_STATES,NUM_STATES,temp7,temp1,covariance);

//printf("\nCovariance\n");
//dump_matrix(NUM_STATES,NUM_STATES,covariance);


//Update state
state->position.x+=corrected_state[0]/k1;
state->position.y+=corrected_state[1]/k2;
state->position.z+=corrected_state[2];
state->velocity=*((vector3_t*)(corrected_state+3));
state->orientation=quaternion_mult(quaternion_axis(*((vector3_t*)(corrected_state+6))),state->orientation);
}

vector3_t rotation_vector_from_quaternion(quaternion_t q)
{
double sin_squared=q.i*q.i+q.j*q.j+q.k*q.k;
	if(sin_squared>0.000001)
	{
	double sin_theta=sqrt(sin_squared);
	double k=2.0*atan2(sin_theta,q.w)/sin_theta;
	return vector3_scale(vector3(q.i,q.j,q.k),k);
	}
return vector3_scale(vector3(q.i,q.j,q.k),2.0);
}

/*
void kalman_smooth(double* states,double* inputs,double* covariances,int num_states)
{
	for(int i=num_states-2;i>0;i--)
	{
	state_t* reference=(state_t*)(states)+i;
	double state_transition_jacobian[NUM_STATES*NUM_STATES];
	double process_noise[NUM_STATES*NUM_STATES];
	compute_state_transition_jacobian(reference,inputs+NUM_INPUTS*i,state_transition_jacobian);
	compute_process_noise(reference,process_noise);

	//Calculate predicted state
	double predicted_state[NUM_STATES];
	predict((state_t*)(states)+i,inputs+NUM_INPUTS*i,predicted_state);
	//Calculate predicted covariance
	double predicted_covariance[NUM_STATES*NUM_STATES];
	double temp1[NUM_STATES*NUM_STATES];
	double temp2[NUM_STATES*NUM_STATES];
	kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,state_transition_jacobian,covariances+i*NUM_STATES*NUM_STATES,temp1);
	//printf("jacobian*covariance\n");
	//dump_matrix(NUM_STATES,NUM_STATES,temp1);


	kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,temp1,state_transition_jacobian,temp2);
	kmatrix_add(NUM_STATES,NUM_STATES,temp2,process_noise,predicted_covariance);

	//printf("Predicted state\n");
	//dump_matrix(NUM_STATES,1,predicted_state);
	//printf("Predicted covariance\n");
	//printf("Predicted covariance\n");
	//dump_matrix(NUM_STATES,NUM_STATES,predicted_covariance);

	//Calculate gain matrix
	double c[NUM_STATES*NUM_STATES];
	memcpy(temp1,predicted_covariance,NUM_STATES*NUM_STATES*sizeof(double));
	//dump_matrix(NUM_STATES,NUM_STATES,temp1);
	kmatrix_inverse(NUM_STATES,temp1);
	//dump_matrix(NUM_STATES,NUM_STATES,temp1);
	kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,covariances+i*NUM_STATES*NUM_STATES,state_transition_jacobian,temp2);
	kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,temp2,temp1,c);

	//printf("Gain\n");
	//dump_matrix(NUM_STATES,NUM_STATES,c);
	
	//Get next state in terms of current reference
	state_t* next_state=(state_t*)states+(i+1);
	vector3_t rotation_difference=rotation_vector_from_quaternion(quaternion_mult(next_state->orientation,quaternion_conjugate(reference->orientation)));	
	double k1=111132.09-566.05*cos(2*reference->position.x)+1.2*cos(4*reference->position.x);
	double k2=111415.13*cos(reference->position.x)-94.55*cos(3*reference->position.x)+0.12*cos(5*reference->position.x);
	double next_state_local[NUM_STATES]={(next_state->position.x-reference->position.x)*k1,(next_state->position.y-reference->position.y)*k2,next_state->position.z-reference->position.z,next_state->velocity.x,next_state->velocity.y,next_state->velocity.z,rotation_difference.x,rotation_difference.y,rotation_difference.z};
	
	//printf("Next state %d\n",i);
	//dump_matrix(NUM_STATES,1,next_state_local);
	
	//Compute difference between next state and predicted state
	double difference[NUM_STATES];
	kmatrix_sub(NUM_STATES,1,next_state_local,predicted_state,difference);
//	printf("Difference state\n");
//	dump_matrix(NUM_STATES,1,difference);
	//Compute updated state
	double temp3[NUM_STATES];
	double state_local[NUM_STATES]={0,0,0,reference->velocity.x,reference->velocity.y,reference->velocity.z,0,0,0};
	//state_local+c*difference
	kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,1,c,difference,temp3);
	double updated_state[NUM_STATES];
	kmatrix_add(NUM_STATES,1,state_local,temp3,updated_state);	

	//printf("Updated state\n");
	//dump_matrix(NUM_STATES,1,updated_state);

	reference->position.x+=updated_state[0]/k1;
	reference->position.y+=updated_state[1]/k2;
	reference->position.z+=updated_state[2];
	reference->velocity=*((vector3_t*)(updated_state+3));
	reference->orientation=quaternion_mult(quaternion_axis(*((vector3_t*)(updated_state+6))),reference->orientation);
	//Compute updated covariance
	double covariance_difference[NUM_STATES*NUM_STATES];
	kmatrix_sub(NUM_STATES,NUM_STATES,covariances+(i+1)*NUM_STATES*NUM_STATES,predicted_covariance,covariance_difference);
	kmatrix_mult(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,c,covariance_difference,temp1);
	kmatrix_mult_transpose(NUM_STATES,NUM_STATES,NUM_STATES,NUM_STATES,temp1,c,temp2);
	memcpy(temp1,covariances+i*NUM_STATES*NUM_STATES,NUM_STATES*NUM_STATES*sizeof(double));
	kmatrix_add(NUM_STATES,NUM_STATES,temp1,temp2,covariances+i*NUM_STATES*NUM_STATES);
	}



}*/
