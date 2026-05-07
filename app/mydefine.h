#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "can.h"

#include "struct_typedef.h"
#include "motor_control.h"
#include "gld_can.h"
#include "For_out.h"

#include "ringbuffer.h"
//#include "pid.h"


#include "scheduler.h"
#include "usart_app.h"
//#include "pid_app.h"
#include "yushu_app.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern CAN_HandleTypeDef hcan1;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern struct rt_ringbuffer uart_ringbuffer;

//extern Changer angle_changer[8];
extern MOTOR_can_recv UT_motor_data[8];

extern double leg[8];
extern double leg0,leg1,leg2,leg3,leg4,leg5,leg6,leg7;
extern double kd;
extern RC_Ctl_t RC_Ctl;



#define OS_SUPPORT 	0
#define SYS_CLK 		168

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8; 

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

extern  double motor1_pos;
extern  double motor2_pos;
extern double pos_want;
extern uint8_t flag;
extern uint8_t mode;
