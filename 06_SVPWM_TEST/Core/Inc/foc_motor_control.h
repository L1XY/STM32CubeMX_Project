#ifndef __FOC_MOTOR_CONTROL_H__
#define __FOC_MOTOR_CONTROL_H__
#include "stdint.h"
#include "stdio.h"
#include "stddef.h"
#include "math.h"
#include "debug.h"
#include "stm32f1xx_hal_tim.h"

#define _PI_2               1.57079632679f          /*π/2*/
#define _2PI                6.283185307179586f      /*2π*/
#define _PI_3               1.047197551196597f      /*60° 弧度制*/
#define SQRT_3              1.732050807568877f      /*根号3*/
#define SQRT_3_DIV_2        0.866025403784438f      /*根号3 /2 */
#define _1_DIV_SQRT_3       0.577350269189625f      /*1除根号3*/
#define _2_DIV_SQRT_3       1.154700538379251f      /*2除根号3*/

#define MAXPWM_DUTY_CYCLE   (5000.0f)
#define MAXPWM_CONTROL      (5000.0f)
#define UDC                 (12.0f)


/* 三相坐标系*/
typedef struct
{
    float iu;
    float iv;
    float iw;
} FOC_U_V_W_t;

/* alpha beta 坐标系*/
typedef struct
{
    float alpha;
    float beta;
} FOC_Alpha_Beta_t;

/*d q坐标系*/
typedef struct
{
    float id;
    float iq;
} FOC_D_Q_t;

/*非零矢量作用时间*/
typedef struct
{
    float t0;
    float t1;
    float t2;
} FOC_VectorTime_t;

/*PWM计数器CCR*/
typedef struct
{
    float counter_0;
    float counter_1;
    float counter_2;
} FOC_PWMCounter_t;


extern TIM_HandleTypeDef htim1;
void FOC_ClarkePark_Debug(void);
void FOC_InverseParkInverseClarke_Debug(void);
void FOC_SVPWM_Debug(void);


#endif
