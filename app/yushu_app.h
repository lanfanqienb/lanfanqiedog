#ifndef __YUSHU_APP_H__
#define __YUSHU_APP_H__

#include "mydefine.h"

extern double leg_stand[8];
extern float ff_torque[8];

void motor_waiting(void);

void dog_start(void);
void dog_int(void);
void dog_Go_int(void);

void dog_forward_mid_int(void);
void dog_forward_mid(fp32 add,fp32 add2);
void dog_forward_mid_out(void);
void dog_left_mid_int(void);
void dog_left_mid(fp32 add,fp32 add2);
void dog_left_mid_out(void);
void dog_right_mid_int(void);
void dog_right_mid(fp32 add,fp32 add2);
void dog_right_mid_out(void);
void dog_back_mid_int(void);
void dog_back_mid(fp32 add,fp32 add2);
void dog_back_mid_out(void);

void dog_squat_int(void);
void dog_squat(fp32 add,fp32 add2);
void dog_squat_dir(fp32 add,fp32 add2);
void dog_squat_out(void);
void dog_squat_int_still(void);

void dog_left_squat_int(void);
void dog_left_squat(fp32 add,fp32 add2);
void dog_left_squat_out(void);
void dog_right_squat_int(void);
void dog_right_squat(fp32 add,fp32 add2);
void dog_right_squat_out(void);
void dog_back_squat_int(void);
void dog_back_squat(fp32 add,fp32 add2);
void dog_back_squat_out(void);

void dog_bunny_jump_forward2(void);
void dog_bunny_jump_up(void);
void dog_bunny_jump_forward4(void);
void dog_bunny_jump_force(void);
void dog_bunny_jump_force2(fp32 F,fp32 H);
void dog_bunny_jump_force4(fp32 F,fp32 H);

void dog_double_jump_force(void);
void dog_treble_jump_force(void);
void dog_bunny_jump_force6(fp32 F, fp32 h[8]);

void dog_jump_force10(fp32 F, fp32 h[8]);
void dog_double_jump_force2(void);
void dog_treble_jump_force2(void);

void dog_bunny_jump_forward7(void);

void dog_jump(void);
void dog_jump2(fp32 W1,fp32 W2,fp32 H);
void dog_jump3(void);

void dog_rear_leg(void);
void dog_rear_flip(void);

void dog_forward_rear_leg_int(void);
void dog_forward_rear_leg(fp32 add,fp32 add2);
void dog_forward_rear_leg_out(void);
void dog_forward_rear_int(void);
void dog_forward_rear(fp32 add,fp32 add2);
void dog_forward_rear_out(void);
void dog_left_rear_int(void);
void dog_left_rear(fp32 add,fp32 add2);
void dog_left_rear_out(void);
void dog_right_rear_int(void);
void dog_right_rear(fp32 add,fp32 add2);
void dog_right_rear_out(void);
void dog_rear_mid_int(void);

void dog_rear_mid(fp32 add,fp32 add2);
void dog_rear_mid_out(void);

void dog_forward_bit_still_int(void);
void dog_forward_bit_still(fp32 add,fp32 add2);
void dog_forward_bit_still_out(void);
void dog_left_bit_still_int(void);
void dog_left_bit_still(fp32 add,fp32 add2);
void dog_left_bit_still_out(void);
void dog_right_bit_still_int(void);
void dog_right_bit_still(fp32 add,fp32 add2);
void dog_right_bit_still_out(void);
void dog_back_bit_still_int(void);
void dog_back_bit_still(fp32 add,fp32 add2);
void dog_back_bit_still_out(void);

void dog_forward_bit_int(void);
void dog_forward_bit(fp32 add,fp32 add2);
void dog_forward_bit_dir(fp32 add,fp32 add2);
void dog_forward_bit_out(void);
void dog_left_bit_int(void);
void dog_left_bit(fp32 add,fp32 add2);
void dog_left_bit_out(void);
void dog_right_bit_int(void);
void dog_right_bit(fp32 add,fp32 add2);
void dog_right_bit_out(void);
void dog_back_bit_int(void);
void dog_back_bit(fp32 add,fp32 add2);
void dog_back_bit_out(void);

void dog_stair_int(void);
void dog_stair_int_still(void);
void dog_stair_out(void);
void dog_stair(void);
void dog_bridge(void);


#endif


