#ifndef FIXED_H_INCLUDED
#define FIXED_H_INCLUDED
#include <stdint.h>

typedef struct
{
double x,y,z;
}vector3_t;

typedef struct
{
double x,y;
}vector2_t;

typedef struct
{
double entries[16];
}matrix_t;

typedef struct
{
double w,i,j,k;
}quaternion_t;

vector2_t vector2(double x,double y);
vector2_t vector2_add(vector2_t a,vector2_t b);
vector2_t vector2_sub(vector2_t a,vector2_t b);
vector2_t vector2_scale(vector2_t a,double b);
double vector2_norm(vector2_t);
double vector2_dot(vector2_t a,vector2_t b);

vector3_t vector3(double x,double y,double z);
vector3_t vector3_from_scalar(double a);
vector3_t vector3_add(vector3_t a,vector3_t b);
vector3_t vector3_sub(vector3_t a,vector3_t b);
vector3_t vector3_scale(vector3_t a,double b);
double vector3_magnitude(vector3_t a);
double vector3_dot(vector3_t a,vector3_t b);
vector3_t vector3_normalize(vector3_t a);
vector3_t vector3_cross(vector3_t a,vector3_t b);

quaternion_t quaternion(double w,double i,double j,double k);
quaternion_t quaternion_axis(vector3_t axis);
quaternion_t quaternion_axis_angle(double angle,vector3_t axis);
quaternion_t quaternion_scale(quaternion_t a,double b);
quaternion_t quaternion_conjugate(quaternion_t q);
double quaternion_magnitude(quaternion_t q);
quaternion_t quaternion_normalize(quaternion_t q);
quaternion_t quaternion_add(quaternion_t a,quaternion_t b);
quaternion_t quaternion_mult(quaternion_t a,quaternion_t b);
quaternion_t quaternion_inverse(quaternion_t q);
vector3_t quaternion_vector(quaternion_t q,vector3_t vec);

#define MATRIX_INDEX(matrix,row,col) ((matrix).entries[4*(row)+(col)])
matrix_t matrix(double a,double b,double c,double d,double e,double f,double g,double h,double i,double j,double k,double l,double m,double n,double o,double p);
matrix_t matrix_identity();
double matrix_determinant(matrix_t matrix);
matrix_t matrix_inverse(matrix_t matrix);
matrix_t matrix_transpose(matrix_t matrix);
matrix_t matrix_mult(matrix_t a,matrix_t b);
vector3_t matrix_vector(matrix_t matrix,vector3_t vector);

matrix_t matrix_translate(vector3_t a);
matrix_t matrix_rotate_x(double angle);
matrix_t matrix_rotate_y(double angle);
matrix_t matrix_rotate_z(double angle);
matrix_t matrix_rotate(quaternion_t q);

#endif // FIXED_H_INCLUDED
