#include "yushu_app.h"
#include "For_out.h"

double ONE_CIRCLE =40.0000000; 

double leg[8]={0,0,0,0,0,0,0,0};
double leg0,leg1,leg2,leg3,leg4,leg5,leg6,leg7;
double kd,flag_1=0,flag_2=0,b=0,b2=0,c=0,c2=0,d=0,d2=0;;
double leg_int[8]={3.0336,1.7061,2.0626,0.2178,2.3067,3.6850,0.6433,5.5677};//
double leg_middle[8]={-6.4249,-5.2640,9.2395,9.4794,11.8030,10.8468,-6.0914,-4.3599};//
double leg_stand[8]={-9.6249,-8.4640,12.4395,12.6794,15.0030,14.0468,-9.2914,-7.5599};
//9.23650265  10.4004
float ff_torque[8]={0,0,0,0,0,0,0,0};  // 每条腿前馈力矩，0=平地，>0=斜坡支撑腿
float slope_ff=0.0f;                    // 斜坡前馈值，调试时修改此值（单位N·m）

/**************************************等待接收到所有电机回传数据**************************************/
void motor_waiting(void)
{
		while (UT_motor_data[0].Pos == 0 || UT_motor_data[1].Pos == 0 || UT_motor_data[2].Pos == 0 || UT_motor_data[3].Pos == 0 || UT_motor_data[4].Pos == 0 || UT_motor_data[5].Pos == 0 || UT_motor_data[6].Pos == 0 || UT_motor_data[7].Pos == 0)
    {
        HAL_Delay(2);
    }
}
/**************************************启动函数**************************************/
void dog_start(void)
{
		leg0=leg_int[0];leg1=leg_int[1];leg2=leg_int[2];leg3=leg_int[3];//0.7370  ,4.6318
	  leg4=leg_int[4];leg5=leg_int[5];leg6=leg_int[6];leg7=leg_int[7];
	  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
	  leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	  
	  leg0=leg_middle[0];leg1=leg_middle[1];leg2=leg_middle[2];leg3=leg_middle[3];//-8.5979,-3.2397
	  leg4=leg_middle[4];leg5=leg_middle[5];leg6=leg_middle[6];leg7=leg_middle[7];
	  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
	  leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	  
	  
	  leg0=leg_middle[0]-3.2;leg1=leg_middle[1]-3.2;leg2=leg_middle[2]+3.2;leg3=leg_middle[3]+3.2;
	  leg4=leg_middle[4]+3.2;leg5=leg_middle[5]+3.2;leg6=leg_middle[6]-3.2;leg7=leg_middle[7]-3.2;
	  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
	  leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	  kd=0.5;
}
/**************************************站立/初始化函数**************************************/
void dog_int(void)
{
	  leg0=leg_middle[0]-3.2;leg1=leg_middle[1]-3.2;leg2=leg_middle[2]+3.2;leg3=leg_middle[3]+3.2;
	  leg4=leg_middle[4]+3.2;leg5=leg_middle[5]+3.2;leg6=leg_middle[6]-3.2;leg7=leg_middle[7]-3.2;
	  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
	  leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	  kd=0.5;	
}
void dog_Go_int(void)
{
	for(fp32 i=0;i<=3;i+=0.00015f){leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}for(fp32 i=0;i<=2;i+=0.00001f);
	for(fp32 i=0;i<=3;i+=0.00015f){leg[4]=leg4-3+i;leg[5]=leg5-3+i;leg[6]=leg6+3-i;leg[7]=leg7+3-i;}for(fp32 i=0;i<=2;i+=0.00001f);
	for(fp32 i=0;i<=3;i+=0.00015f){leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;}for(fp32 i=0;i<=2;i+=0.00001f);
	for(fp32 i=0;i<=3;i+=0.00015f){leg[0]=leg0+3-i;leg[1]=leg1+3-i;leg[2]=leg2-3+i;leg[3]=leg3-3+i;}for(fp32 i=0;i<=2;i+=0.00001f);	
}

/**************************************中前进**************************************/
void dog_forward_mid_int(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00006f){
	leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_forward_mid(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=5;i>0;i-=add2){
		leg[0]=leg0+2.5f-i*0.6f;leg[1]=leg1-2.5f+i;
		leg[2]=leg2-2.5f+i*0.6f;leg[3]=leg3+2.5f-i;
		if(flag_1==0){
			leg[4]=leg4-2.5f-b;leg[5]=leg5+2.5f-b;
			leg[6]=leg6+2.5f+b;leg[7]=leg7-2.5f+b;
			b+=add;if(b>=2.8f)b=2.8f;if(b>=2.8f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4-2.5f-2.8f+c*0.6f;leg[5]=leg5+2.5f-2.8f-c;
			leg[6]=leg6+2.5f+2.8f-c*0.6f;leg[7]=leg7-2.5f+2.8f+c;
			c+=add;if(c>=5)c=5;if(c>=5)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4-2.5f+d-2.8f+5*0.6f;leg[5]=leg5+2.5f+d-2.8f-5;
			leg[6]=leg6+2.5f-d+2.8f-5*0.6f;leg[7]=leg7-2.5f-d+2.8f+5;
			d+=add;if(d>=2.8f)d=2.8f;if(d>=2.8f)flag_1=4;}}
	for(fp32 i2=5;i2>0;i2-=add2){
		leg[4]=leg4-2.5f+i2*0.6f;leg[5]=leg5+2.5f-i2;
		leg[6]=leg6+2.5f-i2*0.6f;leg[7]=leg7-2.5f+i2;
		if(flag_2==0){
			leg[0]=leg0+2.5f+b2;leg[1]=leg1-2.5f+b2;
			leg[2]=leg2-2.5f-b2;leg[3]=leg3+2.5f-b2;
			b2+=add;if(b2>=2.8f)b2=2.8f;if(b2>=2.8f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+2.5f+2.8f-c2*0.6f;leg[1]=leg1-2.5f+2.8f+c2;
			leg[2]=leg2-2.5f-2.8f+c2*0.6f;leg[3]=leg3+2.5f-2.8f-c2;
			c2+=add;if(c2>=5)c2=5;if(c2>=5)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+2.5f-d2+2.8f-5*0.6f;leg[1]=leg1-2.5f-d2+2.8f+5;
			leg[2]=leg2-2.5f+d2-2.8f+5*0.6f;leg[3]=leg3+2.5f+d2-2.8f-5;
			d2+=add;if(d2>=2.8f)d2=2.8f;if(d2>=2.8f)flag_2=4;}}
}
void dog_forward_mid_out(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00006f){
	leg[0]=leg0-2.5f*0.2f+i*0.2f;leg[1]=leg1+2.5f-i;leg[2]=leg2+2.5f*0.2f-i*0.2f;leg[3]=leg3-2.5f+i;
	leg[4]=leg4-2.5f+i;leg[5]=leg5+2.5f-i;leg[6]=leg6+2.5f-i;leg[7]=leg7-2.5f+i;}
}
/**************************************中左转**************************************/
void dog_left_mid_int(void)
{
	for(fp32 i=0;i<=2.2f;i+=0.00006f){leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2-i;
	leg[3]=leg3+i*0.2f;leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_left_mid(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i = 4.4f;i>0;i-=add2){
		leg[0]=leg0+2.2f-i*0.6f;leg[1]=leg1-2.2f+i;
		leg[2]=leg2+2.2f-i;leg[3]=leg3-2.2f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+2.2f-b;leg[5]=leg5-2.2f-b;
			leg[6]=leg6+2.2f+b;leg[7]=leg7-2.2f+b;
			b+=add;if(b>=2.8f)b=2.8f;if(b>=2.8f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+2.2f-2.8f-c;leg[5]=leg5-2.2f-2.8f+c*0.6f;
			leg[6]=leg6+2.2f+2.8f-c*0.6f;leg[7]=leg7-2.2f+2.8f+c;
			c+=add;if(c>=4.4f)c=4.4f;if(c>=4.4f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4+2.2f-2.8f+d-4.4f;leg[5]=leg5-2.2f-2.8f+d+4.4f*0.6f;
			leg[6]=leg6+2.2f+2.8f-d-4.4f*0.6f;leg[7]=leg7-2.2f+2.8f-d+4.4f;
			d+=add;if(d>=2.8f)d=2.8f;if(d>=2.8f)flag_1=4;}}
	for(fp32 i2=4.4f;i2>0;i2-=add2){
		leg[4]=leg4+2.2f-i2;leg[5]=leg5-2.2f+i2*0.6f;
		leg[6]=leg6+2.2f-i2*0.6f;leg[7]=leg7-2.2f+i2;
		if(flag_2==0){
			leg[0]=leg0+2.2f+b2;leg[1]=leg1-2.2f+b2;
			leg[2]=leg2+2.2f-b2;leg[3]=leg3-2.2f-b2;
			b2+=add;if(b2>=2.8f)b2=2.8f;if(b2>=2.8f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+2.2f+2.8f-c2*0.6f;leg[1]=leg1-2.2f+2.8f+c2;
			leg[2]=leg2+2.2f-2.8f-c2;leg[3]=leg3-2.2f-2.8f+c2*0.6f;
			c2+=add;if(c2>4.4f)c2=4.4f;if(c2>=4.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+2.2f+2.8f-d2-4.4f*0.6f;leg[1]=leg1-2.2f+2.8f-d2+4.4f;
			leg[2]=leg2+2.2f-2.8f+d2-4.4f;leg[3]=leg3-2.2f-2.8f+d2+4.4f*0.6f;
			d2+=add;if(d2>=2.8f)d2=2.8f;if(d2>=2.8f)flag_2=4;}}
}
void dog_left_mid_out(void)
{
	for(fp32 i=0;i<=2.2f;i+=0.00012f){
	leg[0]=leg0-2.2f*0.2f+i*0.2f;leg[1]=leg1+2.2f-i;leg[2]=leg2-2.2f+i;leg[3]=leg3+2.2f*0.2f-i*0.2f;
	leg[4]=leg4+2.2f-i;leg[5]=leg5-2.2f+i;leg[6]=leg6+2.2f-i;leg[7]=leg7-2.2f+i;}
}
/**************************************中右转**************************************/
void dog_right_mid_int(void)
{
	for(fp32 i=0;i<=2.2f;i+=0.00006f){
		leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6-i;leg[7]=leg7+i;}
		kd = 0.5f;
}
void dog_right_mid(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=4.4f;i>0;i-=add2){
		leg[0]=leg0 - 2.2f + i;
		leg[1]=leg1 + 2.2f - i*0.6f;
		leg[2]=leg2 - 2.2f + i*0.6f;
		leg[3]=leg3 + 2.2f - i;
		if(flag_1==0){
			leg[4]=leg4 - 2.2f - b;
			leg[5]=leg5 + 2.2f - b;
			leg[6]=leg6 - 2.2f + b;
			leg[7]=leg7 + 2.2f + b;
			b+=add;if(b>=2.8f)b=2.8f;if(b>=2.8f)flag_1=1;
		}
		if(flag_1==1){
			leg[4]=leg4 - 2.2f - 2.8f + c*0.6f;
			leg[5]=leg5 + 2.2f - 2.8f - c;
			leg[6]=leg6 - 2.2f + 2.8f + c;
			leg[7]=leg7 + 2.2f + 2.8f - c*0.6f;
			c+=add;if(c>=4.4f)c=4.4f;if(c>=4.4f)flag_1=2;
		}
		if(flag_1==2){	
			leg[4]=leg4 - 2.2f - 2.8f + d + 4.4f*0.6f;
			leg[5]=leg5 + 2.2f - 2.8f + d - 4.4f;
			leg[6]=leg6 - 2.2f + 2.8f - d + 4.4f;
			leg[7]=leg7 + 2.2f + 2.8f - d - 4.4f*0.6f;
			d+=add;if(d>=2.8f)d=2.8f;if(d>=2.8f)flag_1=4;
		}
	}
	for(fp32 i2=4.4f;i2>0;i2-=add2){
		leg[4]=leg4 - 2.2f + i2*0.6f;
		leg[5]=leg5 + 2.2f - i2;
		leg[6]=leg6 - 2.2f + i2;
		leg[7]=leg7 + 2.2f - i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0 - 2.2f + b2;
			leg[1]=leg1 + 2.2f + b2;
			leg[2]=leg2 - 2.2f - b2;
			leg[3]=leg3 + 2.2f - b2;
			b2+=add;if(b2>=2.8f)b2=2.8f;if(b2>=2.8f)flag_2=1;
		}
		if(flag_2==1){
			leg[0]=leg0 - 2.2f + 2.8f + c2;
			leg[1]=leg1 + 2.2f + 2.8f - c2*0.6f;
			leg[2]=leg2 - 2.2f - 2.8f + c2*0.6f;
			leg[3]=leg3 + 2.2f - 2.8f - c2;
			c2+=add;if(c2>=4.4f)c2=4.4f;if(c2>=4.4f)flag_2=2;
		}
		if(flag_2==2){
			leg[0]=leg0 - 2.2f + 2.8f - d2 + 4.4f;
			leg[1]=leg1 + 2.2f + 2.8f - d2 - 4.4f*0.6f;
			leg[2]=leg2 - 2.2f - 2.8f + d2 + 4.4f*0.6f;
			leg[3]=leg3 + 2.2f - 2.8f + d2 - 4.4f;
			d2+=add;if(d2>=2.8f)d2=2.8f;if(d2>=2.8f)flag_2=4;
		}
	}
}
void dog_right_mid_out(void)
{
	for(fp32 i=0;i<=2.2f;i+=0.00012f){
	leg[0]=leg0+2.2f-i;leg[1]=leg1-2.2f*0.2f+i*0.2f;leg[2]=leg2+2.2f*0.2f-i*0.2f;leg[3]=leg3-2.2f+i;
	leg[4]=leg4-2.2f+i;leg[5]=leg5+2.2f-i;leg[6]=leg6-2.2f+i;leg[7]=leg7+2.2f-i;}
}

/**************************************中后退**************************************/
void dog_back_mid_int(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00006f){
	leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2-i;leg[3]=leg3+i*0.2f;
	leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6-i;leg[7]=leg7+i;}
	kd = 0.5f;
}
void dog_back_mid(fp32 add,fp32 add2)
{
//	dog_back_mid_int();
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=5;i>0;i-=add2){
		leg[0]=leg0-2.5f+i;leg[1]=leg1+2.5f-i*0.6f;
		leg[2]=leg2+2.5f-i;leg[3]=leg3-2.5f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+2.5f-b;leg[5]=leg5-2.5f-b;
			leg[6]=leg6-2.5f+b;leg[7]=leg7+2.5f+b;
			b+=add;if(b>=2.8f)b=2.8f;if(b>=2.8f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+2.5f-2.8f-c;leg[5]=leg5-2.5f-2.8f+c*0.6f;
			leg[6]=leg6-2.5f+2.8f+c;leg[7]=leg7+2.5f+2.8f-c*0.6f;
			c+=add;if(c>=5)c=5;if(c>=5)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4+2.5f+d-2.8f-5;leg[5]=leg5-2.5f+d-2.8f+5*0.6f;
			leg[6]=leg6-2.5f-d+2.8f+5;leg[7]=leg7+2.5f-d+2.8f-5*0.6f;
			d+=add;if(d>=2.8f)d=2.8f;if(d>=2.8f)flag_1=4;}}
	for(fp32 i2=5;i2>0;i2-=add2)
	{
		leg[4]=leg4+2.5f-i2;leg[5]=leg5-2.5f+i2*0.6f;
		leg[6]=leg6-2.5f+i2;leg[7]=leg7+2.5f-i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0-2.5f+b2;leg[1]=leg1+2.5f+b2;
			leg[2]=leg2+2.5f-b2;leg[3]=leg3-2.5f-b2;
			b2+=add;if(b2>=2.8f)b2=2.8f;if(b2>=2.8f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0-2.5f+2.8f+c2;leg[1]=leg1+2.5f+2.8f-c2*0.6f;
			leg[2]=leg2+2.5f-2.8f-c2;leg[3]=leg3-2.5f-2.8f+c2*0.6f;
			c2+=add;if(c2>=5)c2=5;if(c2>=5)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0-2.5f-d2+2.8f+5;leg[1]=leg1+2.5f-d2+2.8f-5*0.6f;
			leg[2]=leg2+2.5f+d2-2.8f-5;leg[3]=leg3-2.5f+d2-2.8f+5*0.6f;
			d2+=add;if(d2>=2.8f)d2=2.8f;if(d2>=2.8f)flag_2=4;}}
//	    dog_back_mid_out();
}
void dog_back_mid_out(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00007f){
	leg[0]=leg0+2.5f-i;leg[1]=leg1-2.5f*0.2f+i*0.2f;leg[2]=leg2-2.5f+i;leg[3]=leg3+2.5f*0.2f-i*0.2f;
	leg[4]=leg4+2.5f-i;leg[5]=leg5-2.5f+i;leg[6]=leg6-2.5f+i;leg[7]=leg7+2.5f-i;}
}


/**************************************蹲前进**************************************/
void dog_squat_int(void)
{
	for(fp32 i=0;i<=6.4f;i+=0.00012f)
	{
			leg0=leg_middle[0]-3.2+i;leg1=leg_middle[1]-3.2+i;leg2=leg_middle[2]+3.2-i;leg3=leg_middle[3]+3.2-i;
	    leg4=leg_middle[4]+3.2-i;leg5=leg_middle[5]+3.2-i;leg6=leg_middle[6]-3.2+i;leg7=leg_middle[7]-3.2+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  kd = 0.5f;
}
void dog_squat_int_still(void)
{	
			leg0=leg_middle[0]-3.2+6.4;leg1=leg_middle[1]-3.2+6.4;leg2=leg_middle[2]+3.2-6.4;leg3=leg_middle[3]+3.2-6.4;
	    leg4=leg_middle[4]+3.2-6.4;leg5=leg_middle[5]+3.2-6.4;leg6=leg_middle[6]-3.2+6.4;leg7=leg_middle[7]-3.2+6.4;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
      kd = 0.5f;
}
void dog_squat(fp32 add,fp32 add2)
{
		flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	  for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0+1.2f-i*0.6f;leg[1]=leg1-1.2f+i;
		leg[2]=leg2-1.2f+i*0.6f;leg[3]=leg3+1.2f-i;
		if(flag_1==0){
			leg[4]=leg4-1.2f-b;leg[5]=leg5+1.2f-b;
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4-1.2f-3.4f+c*0.6f;leg[5]=leg5+1.2f-3.4f-c;
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4-1.2f+d-3.4f+2.4f*0.6f;leg[5]=leg5+1.2f+d-3.4f-2.4f;
			leg[6]=leg6+1.2f-d+3.4f-2.4f*0.6f;leg[7]=leg7-1.2f-d+3.4f+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	  for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4-1.2f+i2*0.6f;leg[5]=leg5+1.2f-i2;
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[0]=leg0+1.2f+b2;leg[1]=leg1-1.2f+b2;
			leg[2]=leg2-1.2f-b2;leg[3]=leg3+1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+1.2f+3.4f-c2*0.6f;leg[1]=leg1-1.2f+3.4f+c2;
			leg[2]=leg2-1.2f-3.4f+c2*0.6f;leg[3]=leg3+1.2f-3.4f-c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+1.2f-d2+3.4f-2.4f*0.6f;leg[1]=leg1-1.2f-d2+3.4f+2.4f;
			leg[2]=leg2-1.2f+d2-3.4f+2.4f*0.6f;leg[3]=leg3+1.2f+d2-3.4f-2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}

// ch3前进时，ch2右偏→右侧缩步（直走往右），ch2左偏→左侧缩步（直走往左），ch2归中→直走
// 右侧腿: leg0,1（电机0,1）和 leg6,7（电机6,7）；左侧腿: leg2,3（电机2,3）和 leg4,5（电机4,5）
void dog_squat_dir(fp32 add, fp32 add2)
{
    // 双向拉伸：一侧缩步(→0.40) + 另一侧阔步(→1.40)，差值最大1.0，效果比单侧缩步翻倍
    // 死区=176（右拨≥1200/左拨≤848），两侧对称
    fp32 comp_right = 1.0f, comp_left = 1.0f;
    const fp32 dz = 176.0f / 660.0f;
    float dev = (float)(RC_Ctl.rc.ch2 - 1024) / 660.0f;
    if (dev > dz) {
        float factor = (dev - dz) / (1.0f - dz);        // 0→1
        comp_right = 1.0f - factor * 0.40f;             // 右侧缩步 1.0→0.60
        comp_left  = 1.0f + factor * 0.20f;             // 左侧阔步 1.0→1.20
        if(comp_right < 0.60f) comp_right = 0.60f;
        if(comp_left  > 1.20f) comp_left  = 1.20f;
    } else if (dev < -dz) {
        float factor = (-dev - dz) / (1.0f - dz);
        comp_left  = 1.0f - factor * 0.40f;             // 左侧缩步 1.0→0.60
        comp_right = 1.0f + factor * 0.20f;             // 右侧阔步 1.0→1.20
        if(comp_left  < 0.60f) comp_left  = 0.60f;
        if(comp_right > 1.20f) comp_right = 1.20f;
    }

    flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
    for(fp32 i=2.4f;i>0;i-=add2){
        leg[0]=(1.2f-i*0.6f)*comp_right+leg0;  leg[1]=(-1.2f+i)*comp_right+leg1;   // 右侧
        leg[2]=(-1.2f+i*0.6f)*comp_left+leg2;  leg[3]=(1.2f-i)*comp_left+leg3;     // 左侧
        if(flag_1==0){
            leg[4]=(-1.2f-b)*comp_left+leg4;             leg[5]=(1.2f-b)*comp_left+leg5;           // 左侧
            leg[6]=(1.2f+b)*comp_right+leg6;             leg[7]=(-1.2f+b)*comp_right+leg7;         // 右侧
            b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
        if(flag_1==1){
            leg[4]=(-1.2f-3.4f+c*0.6f)*comp_left+leg4;  leg[5]=(1.2f-3.4f-c)*comp_left+leg5;
            leg[6]=(1.2f+3.4f-c*0.6f)*comp_right+leg6;  leg[7]=(-1.2f+3.4f+c)*comp_right+leg7;
            c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
        if(flag_1==2){
            leg[4]=(-1.2f+d-3.4f+2.4f*0.6f)*comp_left+leg4;    leg[5]=(1.2f+d-3.4f-2.4f)*comp_left+leg5;
            leg[6]=(1.2f-d+3.4f-2.4f*0.6f)*comp_right+leg6;    leg[7]=(-1.2f-d+3.4f+2.4f)*comp_right+leg7;
            d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
    for(fp32 i2=2.4f;i2>0;i2-=add2){
        leg[4]=(-1.2f+i2*0.6f)*comp_left+leg4;   leg[5]=(1.2f-i2)*comp_left+leg5;   // 左侧
        leg[6]=(1.2f-i2*0.6f)*comp_right+leg6;   leg[7]=(-1.2f+i2)*comp_right+leg7;  // 右侧
        if(flag_2==0){
            leg[0]=(1.2f+b2)*comp_right+leg0;            leg[1]=(-1.2f+b2)*comp_right+leg1;        // 右侧
            leg[2]=(-1.2f-b2)*comp_left+leg2;            leg[3]=(1.2f-b2)*comp_left+leg3;          // 左侧
            b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
        if(flag_2==1){
            leg[0]=(1.2f+3.4f-c2*0.6f)*comp_right+leg0;  leg[1]=(-1.2f+3.4f+c2)*comp_right+leg1;
            leg[2]=(-1.2f-3.4f+c2*0.6f)*comp_left+leg2;  leg[3]=(1.2f-3.4f-c2)*comp_left+leg3;
            c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
        if(flag_2==2){
            leg[0]=(1.2f-d2+3.4f-2.4f*0.6f)*comp_right+leg0;  leg[1]=(-1.2f-d2+3.4f+2.4f)*comp_right+leg1;
            leg[2]=(-1.2f+d2-3.4f+2.4f*0.6f)*comp_left+leg2;  leg[3]=(1.2f+d2-3.4f-2.4f)*comp_left+leg3;
            d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}

void dog_squat_out(void)
{
	for(fp32 i=0;i<=6.4f;i+=0.00012f)
	{
			leg0=leg_middle[0]+3.2-i;leg1=leg_middle[1]+3.2-i;leg2=leg_middle[2]-3.2+i;leg3=leg_middle[3]-3.2+i;
	    leg4=leg_middle[4]-3.2+i;leg5=leg_middle[5]-3.2+i;leg6=leg_middle[6]+3.2-i;leg7=leg_middle[7]+3.2-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	kd = 0.5f;
}	
/**************************************蹲左转**************************************/
void dog_left_squat_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2-i;
	leg[3]=leg3+i*0.2f;leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_left_squat(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0+1.2f-i*0.6f;leg[1]=leg1-1.2f+i;
		leg[2]=leg2+1.2f-i;leg[3]=leg3-1.2f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+1.2f-b;leg[5]=leg5-1.2f-b;
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+1.2f-3.4f-c;leg[5]=leg5-1.2f-3.4f+c*0.6f;
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4+1.2f-3.4f+d-2.4f;leg[5]=leg5-1.2f-3.4f+d+2.4f*0.6f;
			leg[6]=leg6+1.2f+3.4f-d-2.4f*0.6f;leg[7]=leg7-1.2f+3.4f-d+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4+1.2f-i2;leg[5]=leg5-1.2f+i2*0.6f;
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[0]=leg0+1.2f+b2;leg[1]=leg1-1.2f+b2;
			leg[2]=leg2+1.2f-b2;leg[3]=leg3-1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+1.2f+3.4f-c2*0.6f;leg[1]=leg1-1.2f+3.4f+c2;
			leg[2]=leg2+1.2f-3.4f-c2;leg[3]=leg3-1.2f-3.4f+c2*0.6f;
			c2+=add;if(c2>2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+1.2f+3.4f-d2-2.4f*0.6f;leg[1]=leg1-1.2f+3.4f-d2+2.4f;
			leg[2]=leg2+1.2f-3.4f+d2-2.4f;leg[3]=leg3-1.2f-3.4f+d2+2.4f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_left_squat_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0-1.2f*0.2f+i*0.2f;leg[1]=leg1+1.2f-i;leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[4]=leg4+1.2f-i;leg[5]=leg5-1.2f+i;leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f+i;}
}
/**************************************蹲右转**************************************/
void dog_right_squat_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
		leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6-i;leg[7]=leg7+i;}
		kd = 0.5f;
}
void dog_right_squat(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0 - 1.2f + i;
		leg[1]=leg1 + 1.2f - i*0.6f;
		leg[2]=leg2 - 1.2f + i*0.6f;
		leg[3]=leg3 + 1.2f - i;
		if(flag_1==0){
			leg[4]=leg4 - 1.2f - b;
			leg[5]=leg5 + 1.2f - b;
			leg[6]=leg6 - 1.2f + b;
			leg[7]=leg7 + 1.2f + b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4 - 1.2f - 3.4f + c*0.6f;
			leg[5]=leg5 + 1.2f - 3.4f - c;
			leg[6]=leg6 - 1.2f + 3.4f + c;
			leg[7]=leg7 + 1.2f + 3.4f - c*0.6f;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4 - 1.2f - 3.4f + d + 2.4f*0.6f;
			leg[5]=leg5 + 1.2f - 3.4f + d - 2.4f;
			leg[6]=leg6 - 1.2f + 3.4f - d + 2.4f;
			leg[7]=leg7 + 1.2f + 3.4f - d - 2.4f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4 - 1.2f + i2*0.6f;
		leg[5]=leg5 + 1.2f - i2;
		leg[6]=leg6 - 1.2f + i2;
		leg[7]=leg7 + 1.2f - i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0 - 1.2f + b2;
			leg[1]=leg1 + 1.2f + b2;
			leg[2]=leg2 - 1.2f - b2;
			leg[3]=leg3 + 1.2f - b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0 - 1.2f + 3.4f + c2;
			leg[1]=leg1 + 1.2f + 3.4f - c2*0.6f;
			leg[2]=leg2 - 1.2f - 3.4f + c2*0.6f;
			leg[3]=leg3 + 1.2f - 3.4f - c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0 - 1.2f + 3.4f - d2 + 2.4f;
			leg[1]=leg1 + 1.2f + 3.4f - d2 - 2.4f*0.6f;
			leg[2]=leg2 - 1.2f - 3.4f + d2 + 2.4f*0.6f;
			leg[3]=leg3 + 1.2f - 3.4f + d2 - 2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_right_squat_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+1.2f-i;leg[1]=leg1-1.2f*0.2f+i*0.2f;leg[2]=leg2+1.2f*0.2f-i*0.2f;leg[3]=leg3-1.2f+i;
	leg[4]=leg4-1.2f+i;leg[5]=leg5+1.2f-i;leg[6]=leg6-1.2f+i;leg[7]=leg7+1.2f-i;}
}
/**************************************蹲后退**************************************/
void dog_back_squat_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2-i;leg[3]=leg3+i*0.2f;
	leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6-i;leg[7]=leg7+i;}
	kd = 0.5f;
}
void dog_back_squat(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0-1.2f+i;leg[1]=leg1+1.2f-i*0.6f;
		leg[2]=leg2+1.2f-i;leg[3]=leg3-1.2f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+1.2f-b;leg[5]=leg5-1.2f-b;
			leg[6]=leg6-1.2f+b;leg[7]=leg7+1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+1.2f-3.4f-c;leg[5]=leg5-1.2f-3.4f+c*0.6f;
			leg[6]=leg6-1.2f+3.4f+c;leg[7]=leg7+1.2f+3.4f-c*0.6f;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4+1.2f+d-3.4f-2.4f;leg[5]=leg5-1.2f+d-3.4f+2.4f*0.6f;
			leg[6]=leg6-1.2f-d+3.4f+2.4f;leg[7]=leg7+1.2f-d+3.4f-2.4f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4+1.2f-i2;leg[5]=leg5-1.2f+i2*0.6f;
		leg[6]=leg6-1.2f+i2;leg[7]=leg7+1.2f-i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0-1.2f+b2;leg[1]=leg1+1.2f+b2;
			leg[2]=leg2+1.2f-b2;leg[3]=leg3-1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0-1.2f+3.4f+c2;leg[1]=leg1+1.2f+3.4f-c2*0.6f;
			leg[2]=leg2+1.2f-3.4f-c2;leg[3]=leg3-1.2f-3.4f+c2*0.6f;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0-1.2f-d2+3.4f+2.4f;leg[1]=leg1+1.2f-d2+3.4f-2.4f*0.6f;
			leg[2]=leg2+1.2f+d2-3.4f-2.4f;leg[3]=leg3-1.2f+d2-3.4f+2.4f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_back_squat_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+1.2f-i;leg[1]=leg1-1.2f*0.2f+i*0.2f;leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[4]=leg4+1.2f-i;leg[5]=leg5-1.2f+i;leg[6]=leg6-1.2f+i;leg[7]=leg7+1.2f-i;}
}
/**************************************兔子向前跳跃**************************************/

void dog_bunny_jump_forward4(void)
{
    kd = 0.4;
  	for(float i=0;i<=3.7f;i+=0.00006f){
		leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}HAL_Delay(200);
    for(float i=0;i<=2.7f;i+=0.00004f){
		leg[0]=leg0+3.7+i;leg[1]=leg1+3.7-i;leg[2]=leg2-3.7-i*0.703f;leg[3]=leg3-3.7+i*0.703f;
		leg[4]=leg4-3.7-i;leg[5]=leg5-3.7+i;leg[6]=leg6+3.7+i*0.703f;leg[7]=leg7+3.7-i*0.703f;}	
//	  HAL_Delay(600);kd=18;HAL_Delay(20);
	  HAL_Delay(20);kd=18;HAL_Delay(600);
		
		//四脚起跳
	leg[0]=leg0+3.7+2.7f-16.8f;leg[1]=leg1+3.7-2.7f-16.8f;
	leg[2]=leg2-3.7-1.9f+16.8f;leg[3]=leg3-3.7+1.9f+16.8f;
	leg[4]=leg4-3.7-2.7f+16.8f;leg[5]=leg5-3.7+2.7f+16.8f;
	leg[6]=leg6+3.7+1.9f-16.8f;leg[7]=leg7+3.7-1.9f-16.8f;
	HAL_Delay(200);

		//四脚收	
	kd=0.8f;
	leg[0]=leg0-2.7f;leg[1]=leg1+2.7f;leg[2]=leg2+2.7f;leg[3]=leg3-2.7f;
	leg[4]=leg4+2.7f;leg[5]=leg5-2.7f;leg[6]=leg6-2.7f;leg[7]=leg7+2.7f;
}

void dog_bunny_jump_force(void)
{
    // ① 下蹲 —— 与forward4相同
    kd = 0.4f;
    for(float i=0; i<=3.7f; i+=0.00012f){
        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i; leg[3]=leg3-i;
        leg[4]=leg4-i; leg[5]=leg5-i; leg[6]=leg6+i; leg[7]=leg7+i;
    }
    HAL_Delay(200);

    // ② 蓄力 —— 与forward4相同
    for(float i=0; i<=2.7f; i+=0.00008f){
        leg[0]=leg0+3.7f+i;         leg[1]=leg1+3.7f-i;
        leg[2]=leg2-3.7f-i*0.703f;  leg[3]=leg3-3.7f+i*0.703f;
        leg[4]=leg4-3.7f-i;         leg[5]=leg5-3.7f+i;
        leg[6]=leg6+3.7f+i*0.703f;  leg[7]=leg7+3.7f-i*0.703f;
    }
    HAL_Delay(20);
    kd = 18.0f;
    HAL_Delay(600);

    // ③ 起跳 —— 纯力矩控制
    // 记录蓄力完成时各电机实际角度作为基准
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    // kd=0 纯力矩，施加伸展方向力矩12N·m
    kd = 0.0f;
    ff_torque[0]=-12.5f; ff_torque[1]=-12.5f;
    ff_torque[2]= 12.5f; ff_torque[3]= 12.5f;
    ff_torque[4]= 12.5f; ff_torque[5]= 12.5f;
    ff_torque[6]=-12.5f; ff_torque[7]=-12.5f;

    // 等待：任一电机角度变化超过12.0rad(保护上限)，或超时200ms
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if( (pos0-UT_motor_data[0].Pos)>12.0f || (pos1-UT_motor_data[1].Pos)>12.0f ||
            (UT_motor_data[2].Pos-pos2)>12.0f || (UT_motor_data[3].Pos-pos3)>12.0f ||
            (UT_motor_data[4].Pos-pos4)>12.0f || (UT_motor_data[5].Pos-pos5)>12.0f ||
            (pos6-UT_motor_data[6].Pos)>12.0f || (pos7-UT_motor_data[7].Pos)>12.0f )
        { break; }
    }

    // 清除力矩，短暂过渡
    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    kd = 0.5f;
    HAL_Delay(80);

    // ④ 收腿 —— 与forward4相同
    kd = 0.8f;
    leg[0]=leg0-2.7f; leg[1]=leg1+2.7f; leg[2]=leg2+2.7f; leg[3]=leg3-2.7f;
    leg[4]=leg4+2.7f; leg[5]=leg5-2.7f; leg[6]=leg6-2.7f; leg[7]=leg7+2.7f;
}

void dog_bunny_jump_force2(fp32 F,fp32 H)
{
    // ① 下蹲 —— 与forward4相同
    kd = 0.4f;
    for(float i=0; i<=3.7f; i+=0.00012f){
        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i; leg[3]=leg3-i;
        leg[4]=leg4-i; leg[5]=leg5-i; leg[6]=leg6+i; leg[7]=leg7+i;
    }
    HAL_Delay(200);

    // ② 蓄力 —— 与forward4相同
    for(float i=0; i<=2.7f; i+=0.00008f){
        leg[0]=leg0+3.7f+i;         leg[1]=leg1+3.7f-i;
        leg[2]=leg2-3.7f-i*0.703f;  leg[3]=leg3-3.7f+i*0.703f;
        leg[4]=leg4-3.7f-i;         leg[5]=leg5-3.7f+i;
        leg[6]=leg6+3.7f+i*0.703f;  leg[7]=leg7+3.7f-i*0.703f;
    }
    HAL_Delay(20);
    kd = 10.0f;
    HAL_Delay(600);

    // ③ 起跳 —— 纯力矩控制
    // 记录蓄力完成时各电机实际角度作为基准
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    // kd=0 纯力矩，施加伸展方向力矩12N·m
    kd = 0.0f;
    ff_torque[0]=-F; ff_torque[1]=-F;
    ff_torque[2]= F; ff_torque[3]= F;
    ff_torque[4]= F; ff_torque[5]= F;
    ff_torque[6]=-F; ff_torque[7]=-F;//10.5f

    // 等待：任一电机角度变化超过12.0rad(保护上限)，或超时200ms
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if( (pos0-UT_motor_data[0].Pos)>H || (pos1-UT_motor_data[1].Pos)>H ||
            (UT_motor_data[2].Pos-pos2)>H || (UT_motor_data[3].Pos-pos3)>H ||
            (UT_motor_data[4].Pos-pos4)>H || (UT_motor_data[5].Pos-pos5)>H ||
            (pos6-UT_motor_data[6].Pos)>H || (pos7-UT_motor_data[7].Pos)>H )//12.0f
        { break; }
    }

    // 清除力矩，短暂过渡
    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    kd = 0.5f;
    HAL_Delay(80);

    // ④ 收腿 —— 与forward4相同
    kd = 0.8f;
    leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
	  leg[4]=leg4+2.2f;leg[5]=leg5-2.2f;leg[6]=leg6-2.2f;leg[7]=leg7+2.2f;	
}

void dog_bunny_jump_force4(fp32 F,fp32 H)
{
    // ① 下蹲 —— 与forward4相同
    kd = 0.4f;
    for(float i=0; i<=3.7f; i+=0.00016f){
        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i; leg[3]=leg3-i;
        leg[4]=leg4-i; leg[5]=leg5-i; leg[6]=leg6+i; leg[7]=leg7+i;
    }
    HAL_Delay(100);

    // ② 蓄力 —— 与forward4相同
    for(float i=0; i<=2.7f; i+=0.00012f){
        leg[0]=leg0+3.7f+i;         leg[1]=leg1+3.7f-i;
        leg[2]=leg2-3.7f-i*0.703f;  leg[3]=leg3-3.7f+i*0.703f;
        leg[4]=leg4-3.7f-i;         leg[5]=leg5-3.7f+i;
        leg[6]=leg6+3.7f+i*0.703f;  leg[7]=leg7+3.7f-i*0.703f;
    }
    kd = 3.0f;
    HAL_Delay(400);
    // ③ 起跳 —— 纯力矩控制
    // 记录蓄力完成时各电机实际角度作为基准
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    // kd=0 纯力矩，施加伸展方向力矩12N·m
    kd = 0.0f;
    ff_torque[0]=-F; ff_torque[1]=-F;
    ff_torque[2]= F; ff_torque[3]= F;
    ff_torque[4]= F; ff_torque[5]= F;
    ff_torque[6]=-F; ff_torque[7]=-F;//10.5f

    // 等待：任一电机角度变化超过12.0rad(保护上限)，或超时200ms
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if( (pos0-UT_motor_data[0].Pos)>H || (pos1-UT_motor_data[1].Pos)>H ||
            (UT_motor_data[2].Pos-pos2)>H || (UT_motor_data[3].Pos-pos3)>H ||
            (UT_motor_data[4].Pos-pos4)>H || (UT_motor_data[5].Pos-pos5)>H ||
            (pos6-UT_motor_data[6].Pos)>H || (pos7-UT_motor_data[7].Pos)>H )//12.0f
        { break; }
    }

    // 清除力矩，短暂过渡
    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    HAL_Delay(80);

    // ④ 收腿 —— 与forward4相同
    kd = 0.8f;
    leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
	  leg[4]=leg4+2.2f;leg[5]=leg5-2.2f;leg[6]=leg6-2.2f;leg[7]=leg7+2.2f;	
}
void dog_bunny_jump_force6(fp32 F, fp32 h[8])
{
    // ① 下蹲
    kd = 0.4f;
    for(float i=0; i<=3.7f; i+=0.00016f){
        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i; leg[3]=leg3-i;
        leg[4]=leg4-i; leg[5]=leg5-i; leg[6]=leg6+i; leg[7]=leg7+i;
    }
    HAL_Delay(100);

    // ② 蓄力
    for(float i=0; i<=2.7f; i+=0.00012f){
        leg[0]=leg0+3.7f+i;         leg[1]=leg1+3.7f-i;
        leg[2]=leg2-3.7f-i;         leg[3]=leg3-3.7f+i;
        leg[4]=leg4-3.7f-i*0.851f;  leg[5]=leg5-3.7f+i*0.851f;
        leg[6]=leg6+3.7f+i;         leg[7]=leg7+3.7f-i;
    }
    kd = 3.0f;
    HAL_Delay(400);

    // ③ 起跳 —— 各电机独立阈值，达到后立即清零该电机力矩
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    kd = 0.0f;
    ff_torque[0]=-F;    ff_torque[1]=-F;
    ff_torque[2]= F;    ff_torque[3]= F;
    ff_torque[4]= F;    ff_torque[5]= F;
    ff_torque[6]=-F;    ff_torque[7]=-F;

    uint8_t done = 0;
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if(!(done&0x01) && (pos0-UT_motor_data[0].Pos)>h[0]) { ff_torque[0]=0.0f; done|=0x01; }
        if(!(done&0x02) && (pos1-UT_motor_data[1].Pos)>h[1]) { ff_torque[1]=0.0f; done|=0x02; }
        if(!(done&0x04) && (UT_motor_data[2].Pos-pos2)>h[2]) { ff_torque[2]=0.0f; done|=0x04; }
        if(!(done&0x08) && (UT_motor_data[3].Pos-pos3)>h[3]) { ff_torque[3]=0.0f; done|=0x08; }
        if(!(done&0x10) && (UT_motor_data[4].Pos-pos4)>h[4]) { ff_torque[4]=0.0f; done|=0x10; }
        if(!(done&0x20) && (UT_motor_data[5].Pos-pos5)>h[5]) { ff_torque[5]=0.0f; done|=0x20; }
        if(!(done&0x40) && (pos6-UT_motor_data[6].Pos)>h[6]) { ff_torque[6]=0.0f; done|=0x40; }
        if(!(done&0x80) && (pos7-UT_motor_data[7].Pos)>h[7]) { ff_torque[7]=0.0f; done|=0x80; }
        if(done == 0xFF) break;
    }

    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    HAL_Delay(80);

    // ④ 收腿 —— 与forward4相同
    kd = 0.8f;
		
		
//    leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
//	  leg[4]=leg4+3.0f+2.2f;leg[5]=leg5+3.0f-2.2f;leg[6]=leg6-3.0f-2.2f;leg[7]=leg7-3.0f+2.2f;	
//		  leg0=leg_middle[0]-2.2f-0.6f;leg1=leg_middle[1]-2.2f+0.6f;leg2=leg_middle[2]+2.2f+0.6f;leg3=leg_middle[3]+2.2f-0.6f;
//	    leg4=leg_middle[4]+2.2f+0.6f;leg5=leg_middle[5]+2.2f-0.6f;leg6=leg_middle[6]-2.2f-0.6f;leg7=leg_middle[7]-2.2f+0.6f;
//	    leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//	    leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
    leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
	  leg[4]=leg4+2.2f;leg[5]=leg5-2.2f;leg[6]=leg6-2.2f;leg[7]=leg7+2.2f;

		
		
//		HAL_Delay(600);
//	   for(float i=0; i<=2.2f; i+=0.00012f){
//        leg[0]=leg0-2.2f+i;leg[1]=leg1+2.2f-i;leg[2]=leg2+2.2f-i;leg[3]=leg3-2.2f+i;
//	      leg[4]=leg4+2.2f-i;leg[5]=leg5-2.2f+i;leg[6]=leg6-2.2f+i;leg[7]=leg7+2.2f-i;
//    }
}


void dog_jump_force10(fp32 F, fp32 h[8])
{
    // ① 下蹲
    kd = 0.4f;
    for(float i=0; i<=3.7f; i+=0.00016f){
        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i*0.8648f; leg[3]=leg3-i*0.8648f;
        leg[4]=leg4-i*0.8648f; leg[5]=leg5-i*0.8648f; leg[6]=leg6+i; leg[7]=leg7+i;
    }
    HAL_Delay(100);

    // ② 蓄力
    for(float i=0; i<=2.7f; i+=0.00012f){
        leg[0]=leg0+3.7f-i;         leg[1]=leg1+3.7f+i;
        leg[2]=leg2-3.2f+i;         leg[3]=leg3-3.2f-i;
        leg[4]=leg4-3.2f+i;         leg[5]=leg5-3.2f-i;
        leg[6]=leg6+3.7f-i;         leg[7]=leg7+3.7f+i;
    }
    kd = 3.0f;
    HAL_Delay(400);

    // ③ 起跳 —— 各电机独立阈值，达到后立即清零该电机力矩
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    kd = 0.0f;
    ff_torque[0]=-F; ff_torque[1]=-F;
    ff_torque[2]= F; ff_torque[3]= F;
    ff_torque[4]= F; ff_torque[5]= F;
    ff_torque[6]=-F; ff_torque[7]=-F;

    uint8_t done = 0;
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if(!(done&0x01) && (pos0-UT_motor_data[0].Pos)>h[0]) { ff_torque[0]=0.0f; done|=0x01; }
        if(!(done&0x02) && (pos1-UT_motor_data[1].Pos)>h[1]) { ff_torque[1]=0.0f; done|=0x02; }
        if(!(done&0x04) && (UT_motor_data[2].Pos-pos2)>h[2]) { ff_torque[2]=0.0f; done|=0x04; }
        if(!(done&0x08) && (UT_motor_data[3].Pos-pos3)>h[3]) { ff_torque[3]=0.0f; done|=0x08; }
        if(!(done&0x10) && (UT_motor_data[4].Pos-pos4)>h[4]) { ff_torque[4]=0.0f; done|=0x10; }
        if(!(done&0x20) && (UT_motor_data[5].Pos-pos5)>h[5]) { ff_torque[5]=0.0f; done|=0x20; }
        if(!(done&0x40) && (pos6-UT_motor_data[6].Pos)>h[6]) { ff_torque[6]=0.0f; done|=0x40; }
        if(!(done&0x80) && (pos7-UT_motor_data[7].Pos)>h[7]) { ff_torque[7]=0.0f; done|=0x80; }
        if(done == 0xFF) break;
    }

    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    HAL_Delay(80);

    // ④ 收腿
    kd = 0.8f;
    leg[0]=leg0+2.2f; leg[1]=leg1-2.2f; leg[2]=leg2-2.2f; leg[3]=leg3+2.2f;
    leg[4]=leg4-2.2f; leg[5]=leg5+2.2f; leg[6]=leg6+2.2f; leg[7]=leg7-2.2f;
		
		
		HAL_Delay(600);
	   for(float i=0; i<=2.2f; i+=0.00012f){
        leg[0]=leg0+2.2f-i;leg[1]=leg1-2.2f+i;leg[2]=leg2-2.2f+i;leg[3]=leg3+2.2f-i;
	      leg[4]=leg4-2.2f+i;leg[5]=leg5+2.2f-i;leg[6]=leg6+2.2f-i;leg[7]=leg7-2.2f+i;
    }
}

void dog_double_jump_force(void)
{
		dog_bunny_jump_force6(8.5f,(fp32[]){9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f});
		HAL_Delay(600);
		dog_bunny_jump_force6(8.5f,(fp32[]){9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f});		
}	
void dog_double_jump_force2(void)
{
		dog_jump_force10(6.5f,(fp32[]){7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f});//8.0f,8.0f,9.0f,9.0f,9.0f,9.0f,8.0f,8.0f
		HAL_Delay(600);
		dog_jump_force10(6.5f,(fp32[]){7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f});//6.0f,6.0f,7.0f,7.0f,6.0f,7.0f,6.0f,6.0f		
}	

void dog_treble_jump_force(void)
{
		dog_bunny_jump_force6(8.5f,(fp32[]){9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f});	
		HAL_Delay(600);
		dog_bunny_jump_force6(8.5f,(fp32[]){9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f});
		HAL_Delay(600);
		dog_bunny_jump_force6(8.5f,(fp32[]){9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f});
}	
void dog_treble_jump_force2(void)
{
		dog_jump_force10(6.5f,(fp32[]){7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f});	//7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f
		HAL_Delay(600);
		dog_jump_force10(6.5f,(fp32[]){7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f});//9.0f,9.0f,10.0f,10.0f,10.0f,10.0f,9.0f,9.0f
		HAL_Delay(600);
		dog_jump_force10(6.5f,(fp32[]){7.0f,7.0f,8.0f,8.0f,8.0f,8.0f,7.0f,7.0f});
}	
void dog_bunny_jump_forward2(void)//
{
    kd = 0.4;
  	for(float i=0;i<=3.7f;i+=0.00012f){
		leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}HAL_Delay(200);
    for(float i=0;i<=4.4f;i+=0.00008f){
		leg[0]=leg0+3.7+i;leg[1]=leg1+3.7-i;leg[2]=leg2-3.7-i;leg[3]=leg3-3.7+i;
		leg[4]=leg4-3.7-i;leg[5]=leg5-3.7+i;leg[6]=leg6+3.7+i;leg[7]=leg7+3.7-i;}	
	  HAL_Delay(600);kd=18;HAL_Delay(20);
		
		//四脚起跳
	leg[0]=leg0+3.7+4.4f-16.8f;leg[1]=leg1+3.7-4.4f-16.8f;
	leg[2]=leg2-3.7-4.4f+16.8f;leg[3]=leg3-3.7+4.4f+16.8f;
	leg[4]=leg4-3.7-4.4f+16.8f;leg[5]=leg5-3.7+4.4f+16.8f;
	leg[6]=leg6+3.7+4.4f-16.8f;leg[7]=leg7+3.7-4.4f-16.8f;
	HAL_Delay(200);

		//四脚收	
	kd=0.8f;
	leg[0]=leg0-2.7f;leg[1]=leg1+2.7f;leg[2]=leg2+2.7f;leg[3]=leg3-2.7f;
	leg[4]=leg4+2.7f;leg[5]=leg5-2.7f;leg[6]=leg6-2.7f;leg[7]=leg7+2.7f;	
}

void dog_bunny_jump_forward7(void)
{
    kd = 0.4;
  	for(float i=0;i<=3.7f;i+=0.00006f){
		leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}HAL_Delay(200);
    for(float i=0;i<=2.2f;i+=0.00004f){
		leg[0]=leg0+3.7+i;leg[1]=leg1+3.7-i;leg[2]=leg2-3.7-i;leg[3]=leg3-3.7+i;
		leg[4]=leg4-3.7-i;leg[5]=leg5-3.7+i;leg[6]=leg6+3.7+i;leg[7]=leg7+3.7-i;}	
	  HAL_Delay(20);kd=18;HAL_Delay(600);
		
		//四脚起跳
		leg[0]=leg0+3.7+2.2f-16.8f;leg[1]=leg1+3.7-2.2f-16.8f;
		leg[2]=leg2-3.7-2.2f+16.8f;leg[3]=leg3-3.7+2.2f+16.8f;
		leg[4]=leg4-3.7-2.2f+16.8f;leg[5]=leg5-3.7+2.2f+16.8f;
		leg[6]=leg6+3.7+2.2f-16.8f;leg[7]=leg7+3.7-2.2f-16.8f;
		HAL_Delay(200);

		//四脚收	
		kd=0.8f;
//		leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
//		leg[4]=leg4+2.2f;leg[5]=leg5-2.2f;leg[6]=leg6-2.2f;leg[7]=leg7+2.2f;
		leg[0]=leg0-2.2f;leg[1]=leg1+2.2f;leg[2]=leg2+2.2f;leg[3]=leg3-2.2f;
		leg[4]=leg4+2.2f;leg[5]=leg5-2.2f;leg[6]=leg6-2.2f;leg[7]=leg7+2.2f;
		
}

void dog_jump(void)
{
		 
	  for(float i=0;i<=0.7f;i+=0.00006f){
		leg[0]=leg0+i;leg[1]=leg1-i;leg[2]=leg2-i;leg[3]=leg3+i;
		leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6+i;leg[7]=leg7-i;}HAL_Delay(200);
	   
	   for(fp32 i=0;i<=8.6f;i+=0.00008f)
		 {
				leg[0]=leg0+0.7f-i;leg[1]=leg1-0.7f-i;leg[2]=leg2-0.7f-i*0.13953f;leg[3]=leg3+0.7f-i*0.13953f;
				leg[4]=leg4-0.7f+i;leg[5]=leg5+0.7f+i;leg[6]=leg6+0.7f+i*0.13953f;leg[7]=leg7-0.7f+i*0.13953f;
     } kd = 0.5f;
		 
		kd=3;HAL_Delay(3000);
		 
		 //四脚起跳
		leg[0]=leg0+0.7f-8.6f-3.0f;leg[1]=leg1+0.7f-8.6f-3.0f;
		leg[2]=leg2-0.7f-1.2f+6.8f;leg[3]=leg3-0.7f-1.2f+6.8f;
		leg[4]=leg4-0.7f+8.6f+3.0f;leg[5]=leg5-0.7f+8.6f+3.0f;
		leg[6]=leg6+0.7f+1.2f-6.8f;leg[7]=leg7+0.7f+1.2f-6.8f;
		HAL_Delay(200);
		 
		leg[0]=leg0-3.2f;leg[1]=leg1-3.2f;leg[2]=leg2+3.2f;leg[3]=leg3+3.2f;
		leg[4]=leg4+3.2f;leg[5]=leg5+3.2f;leg[6]=leg6-3.2f;leg[7]=leg7-3.2f;	 
}	

void dog_jump2(fp32 W1,fp32 W2,fp32 H)
{
		fp32 b= W2/W1;
    	
	  for(float i=0;i<=W1;i+=0.00006f){
		leg[0]=leg0+i*b;leg[1]=leg1-i*b;leg[2]=leg2-i;leg[3]=leg3+i;
		leg[4]=leg4-i*b;leg[5]=leg5+i*b;leg[6]=leg6+i;leg[7]=leg7-i;}HAL_Delay(200);
	   
		 
//		kd=2;
		HAL_Delay(2000);
		 
		 //四脚起跳
		leg[0]=leg0+W2-H;leg[1]=leg1-W2-H;
		leg[2]=leg2-W1+H;leg[3]=leg3+W1+H;
		leg[4]=leg4-W2+H;leg[5]=leg5+W2+H;
		leg[6]=leg6+W1-H;leg[7]=leg7-W1-H;
		HAL_Delay(200);
		 
//		leg[0]=leg0-1.2f;leg[1]=leg1+1.2f;leg[2]=leg2+1.2f;leg[3]=leg3-1.2f;
//		leg[4]=leg4+1.2f;leg[5]=leg5-1.2f;leg[6]=leg6-1.2f;leg[7]=leg7+1.2f;	 
//		leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//		leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
}	

void dog_jump3(void)
{
		 	   
	   for(fp32 i=0;i<=8.6f;i+=0.00008f)
		 {
				leg[0]=leg0-i;leg[1]=leg1-i;leg[2]=leg2-i*0.13953f;leg[3]=leg3-i*0.13953f;
				leg[4]=leg4+i;leg[5]=leg5+i;leg[6]=leg6+i*0.13953f;leg[7]=leg7+i*0.13953f;
     } kd = 0.5f;
		 
		kd=3;HAL_Delay(3000);
		 
		 //四脚起跳
		leg[0]=leg0-8.6f-3.0f;leg[1]=leg1-8.6f-3.0f;
		leg[2]=leg2-1.2f+6.8f;leg[3]=leg3-1.2f+6.8f;
		leg[4]=leg4+8.6f+3.0f;leg[5]=leg5+8.6f+3.0f;
		leg[6]=leg6+1.2f-6.8f;leg[7]=leg7+1.2f-6.8f;
		HAL_Delay(200);
		 
		leg[0]=leg0-3.2f-1.2f;leg[1]=leg1-3.2f+1.2f;leg[2]=leg2+3.2f+1.2f;leg[3]=leg3+3.2f-1.2f;
		leg[4]=leg4+3.2f+1.2f;leg[5]=leg5+3.2f-1.2f;leg[6]=leg6-3.2f-1.2f-1.2f;leg[7]=leg7-3.2f+1.2f;	 
}	

void dog_jump4(fp32 W1,fp32 W2,fp32 F1,fp32 F2,fp32 H)
{
    // ① 下蹲 —— 与forward4相同
    kd = 0.4f;
//    for(float i=0; i<=3.7f; i+=0.00016f){
//        leg[0]=leg0+i; leg[1]=leg1+i; leg[2]=leg2-i; leg[3]=leg3-i;
//        leg[4]=leg4-i; leg[5]=leg5-i; leg[6]=leg6+i; leg[7]=leg7+i;
//    }
//    HAL_Delay(100);

    // ② 蓄力 —— 与forward4相同
    for(float i=0; i<= W1; i+=0.00012f){
        leg[0]=leg0+i;         leg[1]=leg1-i;
        leg[2]=leg2-i;         leg[3]=leg3+i;//前
        leg[4]=leg4-i;         leg[5]=leg5+i;
        leg[6]=leg6+i;         leg[7]=leg7-i;//前
    }
//    kd = 3.0f;
//    HAL_Delay(400);
    // ③ 起跳 —— 纯力矩控制
    // 记录蓄力完成时各电机实际角度作为基准
    float pos0=UT_motor_data[0].Pos; float pos1=UT_motor_data[1].Pos;
    float pos2=UT_motor_data[2].Pos; float pos3=UT_motor_data[3].Pos;
    float pos4=UT_motor_data[4].Pos; float pos5=UT_motor_data[5].Pos;
    float pos6=UT_motor_data[6].Pos; float pos7=UT_motor_data[7].Pos;

    // kd=0 纯力矩，施加伸展方向力矩12N·m
    kd = 0.0f;
    ff_torque[0]=-F2; ff_torque[1]=-F2;
    ff_torque[2]= F1; ff_torque[3]= F1;
    ff_torque[4]= F2; ff_torque[5]= F2;
    ff_torque[6]=-F1; ff_torque[7]=-F1;//10.5f

    // 等待：任一电机角度变化超过12.0rad(保护上限)，或超时200ms
    uint32_t t0 = HAL_GetTick();
    while((HAL_GetTick() - t0) < 200)
    {
        if( (pos0-UT_motor_data[0].Pos)>H || (pos1-UT_motor_data[1].Pos)>H ||
            (UT_motor_data[2].Pos-pos2)>H || (UT_motor_data[3].Pos-pos3)>H ||
            (UT_motor_data[4].Pos-pos4)>H || (UT_motor_data[5].Pos-pos5)>H ||
            (pos6-UT_motor_data[6].Pos)>H || (pos7-UT_motor_data[7].Pos)>H )//12.0f
        { break; }
    }

    // 清除力矩，短暂过渡
    ff_torque[0]=ff_torque[1]=ff_torque[2]=ff_torque[3]=0.0f;
    ff_torque[4]=ff_torque[5]=ff_torque[6]=ff_torque[7]=0.0f;
    HAL_Delay(80);

    // ④ 收腿 —— 与forward4相同
    kd = 0.8f;
    leg[0]=leg0-1.5f;leg[1]=leg1+1.5f;leg[2]=leg2+1.5f;leg[3]=leg3-1.5f;
	  leg[4]=leg4+1.5f;leg[5]=leg5-1.5f;leg[6]=leg6-1.5f;leg[7]=leg7+1.5f;	
		
		HAL_Delay(600);
	   for(float i=0; i<=1.5f; i+=0.00012f){
        leg[0]=leg0-1.5f+i;leg[1]=leg1+1.5f-i;leg[2]=leg2+1.5f-i;leg[3]=leg3-1.5f+i;
	      leg[4]=leg4+1.5f-i;leg[5]=leg5-1.5f+i;leg[6]=leg6-1.5f+i;leg[7]=leg7+1.5f-i;
    }
}

/**************************************卡墙移动**************************************/
void dog_rear_leg(void)
{
	    leg0=leg_middle[0]-3.2+6.4+9;leg1=leg_middle[1]-3.2+6.4-6;leg2=leg_middle[2]+3.2-6.4+2.2f;leg3=leg_middle[3]+3.2-6.4-2.2f;
	    leg4=leg_middle[4]+3.2-6.4-9;leg5=leg_middle[5]+3.2-6.4+6;leg6=leg_middle[6]-3.2+6.4-2.2f;leg7=leg_middle[7]-3.2+6.4+2.2f;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
      kd = 0.5f;
}

void dog_rear_flip(void)
{
	    leg0=leg_middle[0]-3.2+6.4+20;leg1=leg_middle[1]-3.2+6.4-15;leg2=leg_middle[2]+3.2-6.4+2.2f;leg3=leg_middle[3]+3.2-6.4-2.2f;
	    leg4=leg_middle[4]+3.2-6.4-20;leg5=leg_middle[5]+3.2-6.4+15;leg6=leg_middle[6]-3.2+6.4-2.2f;leg7=leg_middle[7]-3.2+6.4+2.2f;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
      kd = 0.5f;
}

void dog_forward_rear_leg_int(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_forward_rear_leg(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=1.2f;i>0;i-=add2){
		leg[2]=leg2-0.6f+i*0.6f;leg[3]=leg3+0.6f-i;
		if(flag_1==0){
			leg[6]=leg6+0.6f+b;leg[7]=leg7-0.6f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[6]=leg6+0.6f+3.4f-c*0.6f;leg[7]=leg7-0.6f+3.4f+c;
			c+=add;if(c>=1.2f)c=1.2f;if(c>=1.2f)flag_1=2;}
		if(flag_1==2){
			leg[6]=leg6+0.6f-d+3.4f-1.2f*0.6f;leg[7]=leg7-0.6f-d+3.4f+1.2f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=1.2f;i2>0;i2-=add2){
		leg[6]=leg6+0.6f-i2*0.6f;leg[7]=leg7-0.6f+i2;
		if(flag_2==0){
			leg[2]=leg2-0.6f-b2;leg[3]=leg3+0.6f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[2]=leg2-0.6f-3.4f+c2*0.6f;leg[3]=leg3+0.6f-3.4f-c2;
			c2+=add;if(c2>=1.2f)c2=1.2f;if(c2>=1.2f)flag_2=2;}
		if(flag_2==2){
			leg[2]=leg2-0.6f+d2-3.4f+1.2f*0.6f;leg[3]=leg3+0.6f+d2-3.4f-1.2f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_forward_rear_leg_out(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[2]=leg2+0.6f*0.2f-i*0.2f;leg[3]=leg3-0.6f+i;
	leg[6]=leg6+0.6f-i;leg[7]=leg7-0.6f + i;}
}
void dog_forward_rear_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_forward_rear(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[2]=leg2-1.2f+i*0.6f;leg[3]=leg3+1.2f-i;
		if(flag_1==0){
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){
			leg[6]=leg6+1.2f-d+3.4f-2.4f*0.6f;leg[7]=leg7-1.2f-d+3.4f+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[2]=leg2-1.2f-b2;leg[3]=leg3+1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[2]=leg2-1.2f-3.4f+c2*0.6f;leg[3]=leg3+1.2f-3.4f-c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[2]=leg2-1.2f+d2-3.4f+2.4f*0.6f;leg[3]=leg3+1.2f+d2-3.4f-2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_forward_rear_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[2]=leg2+1.2f*0.2f-i*0.2f;leg[3]=leg3-1.2f+i;
	leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f + i;}
}

void dog_rear_mid_int(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00006f){
	leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_rear_mid(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=5;i>0;i-=add2){
		leg[2]=leg2-2.5f+i*0.6f;leg[3]=leg3+2.5f-i;
		if(flag_1==0){
			leg[6]=leg6+2.5f+b;leg[7]=leg7-2.5f+b;
			b+=add;if(b>=2.8f)b=2.8f;if(b>=2.8f)flag_1=1;}
		if(flag_1==1){
			leg[6]=leg6+2.5f+2.8f-c*0.6f;leg[7]=leg7-2.5f+2.8f+c;
			c+=add;if(c>=5)c=5;if(c>=5)flag_1=2;}
		if(flag_1==2){
			leg[6]=leg6+2.5f-d+2.8f-5*0.6f;leg[7]=leg7-2.5f-d+2.8f+5;
			d+=add;if(d>=2.8f)d=2.8f;if(d>=2.8f)flag_1=4;}}
	for(fp32 i2=5;i2>0;i2-=add2){
		leg[6]=leg6+2.5f-i2*0.6f;leg[7]=leg7-2.5f+i2;
		if(flag_2==0){
			leg[2]=leg2-2.5f-b2;leg[3]=leg3+2.5f-b2;
			b2+=add;if(b2>=2.8f)b2=2.8f;if(b2>=2.8f)flag_2=1;}
		if(flag_2==1){
			leg[2]=leg2-2.5f-2.8f+c2*0.6f;leg[3]=leg3+2.5f-2.8f-c2;
			c2+=add;if(c2>=5)c2=5;if(c2>=5)flag_2=2;}
		if(flag_2==2){
			leg[2]=leg2-2.5f+d2-2.8f+5*0.6f;leg[3]=leg3+2.5f+d2-2.8f-5;
			d2+=add;if(d2>=2.8f)d2=2.8f;if(d2>=2.8f)flag_2=4;}}
}
void dog_rear_mid_out(void)
{
	for(fp32 i=0;i<=2.5f;i+=0.00006f){
	leg[2]=leg2+2.5f*0.2f-i*0.2f;leg[3]=leg3-2.5f+i;
	leg[6]=leg6+2.5f-i;leg[7]=leg7-2.5f+i;}
}

void dog_left_rear_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){leg[2]=leg2-i;
	leg[3]=leg3+i*0.2f;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_left_rear(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[2]=leg2+1.2f-i;leg[3]=leg3-1.2f+i*0.6f;
		if(flag_1==0){
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[6]=leg6+1.2f+3.4f-d-2.4f*0.6f;leg[7]=leg7-1.2f+3.4f-d+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[2]=leg2+1.2f-b2;leg[3]=leg3-1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[2]=leg2+1.2f-3.4f-c2;leg[3]=leg3-1.2f-3.4f+c2*0.6f;
			c2+=add;if(c2>2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[2]=leg2+1.2f-3.4f+d2-2.4f;leg[3]=leg3-1.2f-3.4f+d2+2.4f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_left_rear_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f+i;}
}
void dog_right_rear_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
		leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
		leg[6]=leg6-i;leg[7]=leg7+i;}
		kd = 0.5f;
}
void dog_right_rear(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[2]=leg2 - 1.2f + i*0.6f;leg[3]=leg3 + 1.2f - i;
		if(flag_1==0){
			leg[6]=leg6 - 1.2f + b;leg[7]=leg7 + 1.2f + b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[6]=leg6 - 1.2f + 3.4f + c;leg[7]=leg7 + 1.2f + 3.4f - c*0.6f;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[6]=leg6 - 1.2f + 3.4f - d + 2.4f;leg[7]=leg7 + 1.2f + 3.4f - d - 2.4f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[6]=leg6 - 1.2f + i2;leg[7]=leg7 + 1.2f - i2*0.6f;
		if(flag_2==0){
			leg[2]=leg2 - 1.2f - b2;leg[3]=leg3 + 1.2f - b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[2]=leg2 - 1.2f - 3.4f + c2*0.6f;leg[3]=leg3 + 1.2f - 3.4f - c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[2]=leg2 - 1.2f - 3.4f + d2 + 2.4f*0.6f;leg[3]=leg3 + 1.2f - 3.4f + d2 - 2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_right_rear_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[2]=leg2+1.2f*0.2f-i*0.2f;leg[3]=leg3-1.2f+i;
	leg[6]=leg6-1.2f+i;leg[7]=leg7+1.2f-i;}
}

/**************************************兔子向上跳跃**************************************/
void dog_bunny_jump_up(void)
{
    kd = 0.4;
  	for(float i=0;i<=3.7f;i+=0.00002f){
		leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}HAL_Delay(200);
    for(float i=0;i<=8.0f;i+=0.00002f){
		leg[0]=leg0+3.7+i;leg[1]=leg1+3.7-i;
		leg[4]=leg4-3.7-i;leg[5]=leg5-3.7+i;}
		
		//前脚跳，站立			
		HAL_Delay(20);kd=18;HAL_Delay(600);
	  leg[2]=leg2-3.7+12;leg[3]=leg3-3.7+12;leg[6]=leg6+3.7-12;leg[7]=leg7+3.7-12;

		HAL_Delay(10);kd=0.6;
		//前脚回，拉高重心	
    HAL_Delay(150);
		leg[2]=leg2-3.7+12-10;leg[3]=leg3-3.7+12-10;leg[6]=leg6+3.7-12+10; leg[7]=leg7+3.7-12+10;
		
		
		//后腿跳
		HAL_Delay(10);kd=18;HAL_Delay(200);
		leg[0]=leg0+3.7+8.0-18;leg[1]=leg1+3.7-8.0-16;leg[4]=leg4-3.7-8.0+18;leg[5]=leg5-3.7+8.0+16;	
		//HAL_Delay(100);		

		//2前腿转，拉高重心	
		kd=0.8;HAL_Delay(230);
    leg[2]=leg2-3.7+12-10+5;leg[3]=leg3-3.7+12-10-5;leg[6]=leg6+3.7-12+10-5; leg[7]=leg7+3.7-12+10+5;		
    leg[0]=leg0+3.7+8.0-18+17;leg[1]=leg1+3.7-8.0-16+15;leg[4]=leg4-3.7-8.0+18-17;leg[5]=leg5-3.7+8.0+16-15;

    kd=0.4f;HAL_Delay(300);

			
//		//后脚回，准备落地
//		leg[0]=leg0+3.7-1.8+6+4;leg[1]=leg1+3.7+1.8-6-4;leg[4]=leg4-3.7+1.8-6-4;leg[5]=leg5-3.7-1.8+6+4;
//		HAL_Delay(100);	
}
/**************************************后空翻**************************************/
void dog_backflip(void)
{
		kd = 0.4;
    for(float i=0;i<=3.7f;i+=0.00002f){
		leg[0]=leg0+i;leg[1]=leg1+i;leg[2]=leg2-i;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7+i;}HAL_Delay(200);
		for(float i=0;i<=8.0f;i+=0.00002f){
		leg[2]=leg0+3.7+i;leg[3]=leg1+3.7-i;
		leg[6]=leg4-3.7-i;leg[7]=leg5-3.7+i;}
		
//		//后脚跳，站立			
//		HAL_Delay(600);kd=18;HAL_Delay(20);
//	  leg[2]=leg2-3.7+12;leg[3]=leg3-3.7+12;leg[6]=leg6+3.7-12;leg[7]=leg7+3.7-12;
}
/***********************************常驻微调前进***********************************/
void dog_forward_bit_still_int(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_forward_bit_still(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=1.2f;i>0;i-=add2){
		leg[0]=leg0+0.6f-i*0.6f;leg[1]=leg1-0.6f+i;
		leg[2]=leg2-0.6f+i*0.6f;leg[3]=leg3+0.6f-i;
		if(flag_1==0){
			leg[4]=leg4-0.6f-b;leg[5]=leg5+0.6f-b;
			leg[6]=leg6+0.6f+b;leg[7]=leg7-0.6f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4-0.6f-3.4f+c*0.6f;leg[5]=leg5+0.6f-3.4f-c;
			leg[6]=leg6+0.6f+3.4f-c*0.6f;leg[7]=leg7-0.6f+3.4f+c;
			c+=add;if(c>=1.2f)c=1.2f;if(c>=1.2f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4-0.6f+d-3.4f+1.2f*0.6f;leg[5]=leg5+0.6f+d-3.4f-1.2f;
			leg[6]=leg6+0.6f-d+3.4f-1.2f*0.6f;leg[7]=leg7-0.6f-d+3.4f+1.2f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=1.2f;i2>0;i2-=add2){
		leg[4]=leg4-0.6f+i2*0.6f;leg[5]=leg5+0.6f-i2;
		leg[6]=leg6+0.6f-i2*0.6f;leg[7]=leg7-0.6f+i2;
		if(flag_2==0){
			leg[0]=leg0+0.6f+b2;leg[1]=leg1-0.6f+b2;
			leg[2]=leg2-0.6f-b2;leg[3]=leg3+0.6f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+0.6f+3.4f-c2*0.6f;leg[1]=leg1-0.6f+3.4f+c2;
			leg[2]=leg2-0.6f-3.4f+c2*0.6f;leg[3]=leg3+0.6f-3.4f-c2;
			c2+=add;if(c2>=1.2f)c2=1.2f;if(c2>=1.2f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+0.6f-d2+3.4f-1.2f*0.6f;leg[1]=leg1-0.6f-d2+3.4f+1.2f;
			leg[2]=leg2-0.6f+d2-3.4f+1.2f*0.6f;leg[3]=leg3+0.6f+d2-3.4f-1.2f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_forward_bit_still_out(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[0]=leg0-0.6f*0.2f+i*0.2f;leg[1]=leg1+0.6f-i;leg[2]=leg2+0.6f*0.2f-i*0.2f;leg[3]=leg3-0.6f+i;
	leg[4]=leg4-0.6f+i;leg[5]=leg5+0.6f-i;leg[6]=leg6+0.6f-i;leg[7]=leg7-0.6f + i;}
}
/**************************************常驻微调左转**************************************/
void dog_left_bit_still_int(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2-i;
	leg[3]=leg3+i*0.2f;leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd=0.5f;
}
void dog_left_bit_still(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i = 1.2f;i>0;i-=add2){
		leg[0]=leg0+0.6f-i*0.6f;leg[1]=leg1-0.6f+i;
		leg[2]=leg2+0.6f-i;leg[3]=leg3-0.6f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+0.6f-b;leg[5]=leg5-0.6f-b;
			leg[6]=leg6+0.6f+b;leg[7]=leg7-0.6f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+0.6f-3.4f-c;leg[5]=leg5-0.6f-3.4f+c*0.6f;
			leg[6]=leg6+0.6f+3.4f-c*0.6f;leg[7]=leg7-0.6f+3.4f+c;
			c+=add;if(c>=1.2f)c=1.2f;if(c>=1.2f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4+0.6f-3.4f+d-1.2f;leg[5]=leg5-0.6f-3.4f+d+1.2f*0.6f;
			leg[6]=leg6+0.6f+3.4f-d-1.2f*0.6f;leg[7]=leg7-0.6f+3.4f-d+1.2f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=1.2f;i2>0;i2-=add2){
		leg[4]=leg4+0.6f-i2;leg[5]=leg5-0.6f+i2*0.6f;
		leg[6]=leg6+0.6f-i2*0.6f;leg[7]=leg7-0.6f+i2;
		if(flag_2==0){
			leg[0]=leg0+0.6f+b2;leg[1]=leg1-0.6f+b2;
			leg[2]=leg2+0.6f-b2;leg[3]=leg3-0.6f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+0.6f+3.4f-c2*0.6f;leg[1]=leg1-0.6f+3.4f+c2;
			leg[2]=leg2+0.6f-3.4f-c2;leg[3]=leg3-0.6f-3.4f+c2*0.6f;
			c2+=add;if(c2>1.2f)c2=1.2f;if(c2>=1.2f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+0.6f+3.4f-d2-1.2f*0.6f;leg[1]=leg1-0.6f+3.4f-d2+1.2f;
			leg[2]=leg2+0.6f-3.4f+d2-1.2f;leg[3]=leg3-0.6f-3.4f+d2+1.2f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_left_bit_still_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0-1.2f*0.2f+i*0.2f;leg[1]=leg1+1.2f-i;leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[4]=leg4+1.2f-i;leg[5]=leg5-1.2f+i;leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f+i;}
}
/**************************************常驻微调右转**************************************/
void dog_right_bit_still_int(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
		leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6-i;leg[7]=leg7+i;}
		kd = 0.5f;
}
void dog_right_bit_still(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=1.2f;i>0;i-=add2){
		leg[0]=leg0 - 0.6f + i;leg[1]=leg1 + 0.6f - i*0.6f;
		leg[2]=leg2 - 0.6f + i*0.6f;leg[3]=leg3 + 0.6f - i;
		if(flag_1==0){
			leg[4]=leg4 - 0.6f - b;leg[5]=leg5 + 0.6f - b;
			leg[6]=leg6 - 0.6f + b;leg[7]=leg7 + 0.6f + b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4 - 0.6f - 3.4f + c*0.6f;leg[5]=leg5 + 0.6f - 3.4f - c;
			leg[6]=leg6 - 0.6f + 3.4f + c;leg[7]=leg7 + 0.6f + 3.4f - c*0.6f;
			c+=add;if(c>=1.2f)c=1.2f;if(c>=1.2f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4 - 0.6f - 3.4f + d + 1.2f*0.6f;leg[5]=leg5 + 0.6f - 3.4f + d - 1.2f;
			leg[6]=leg6 - 0.6f + 3.4f - d + 1.2f;leg[7]=leg7 + 0.6f + 3.4f - d - 1.2f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=1.2f;i2>0;i2-=add2){
		leg[4]=leg4 - 0.6f + i2*0.6f;leg[5]=leg5 + 0.6f - i2;
		leg[6]=leg6 - 0.6f + i2;leg[7]=leg7 + 0.6f - i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0 - 0.6f + b2;leg[1]=leg1 + 0.6f + b2;
			leg[2]=leg2 - 0.6f - b2;leg[3]=leg3 + 0.6f - b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0 - 0.6f + 3.4f + c2;leg[1]=leg1 + 0.6f + 3.4f - c2*0.6f;
			leg[2]=leg2 - 0.6f - 3.4f + c2*0.6f;leg[3]=leg3 + 0.6f - 3.4f - c2;
			c2+=add;if(c2>=1.2f)c2=1.2f;if(c2>=1.2f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0 - 0.6f + 3.4f - d2 + 1.2f;leg[1]=leg1 + 0.6f + 3.4f - d2 - 1.2f*0.6f;
			leg[2]=leg2 - 0.6f - 3.4f + d2 + 1.2f*0.6f;leg[3]=leg3 + 0.6f - 3.4f + d2 - 1.2f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_right_bit_still_out(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[0]=leg0+0.6f-i;leg[1]=leg1-0.6f*0.2f+i*0.2f;leg[2]=leg2+0.6f*0.2f-i*0.2f;leg[3]=leg3-0.6f+i;
	leg[4]=leg4-0.6f+i;leg[5]=leg5+0.6f-i;leg[6]=leg6-0.6f+i;leg[7]=leg7+0.6f-i;}
}
/**************************************常驻微调后退**************************************/
void dog_back_bit_still_int(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2-i;leg[3]=leg3+i*0.2f;
	leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6-i;leg[7]=leg7+i;}
	kd = 0.5f;
}
void dog_back_bit_still(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=1.2f;i>0;i-=add2){
		leg[0]=leg0-0.6f+i;leg[1]=leg1+0.6f-i*0.6f;
		leg[2]=leg2+0.6f-i;leg[3]=leg3-0.6f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+0.6f-b;leg[5]=leg5-0.6f-b;
			leg[6]=leg6-0.6f+b;leg[7]=leg7+0.6f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+0.6f-3.4f-c;leg[5]=leg5-0.6f-3.4f+c*0.6f;
			leg[6]=leg6-0.6f+3.4f+c;leg[7]=leg7+0.6f+3.4f-c*0.6f;
			c+=add;if(c>=1.2f)c=1.2f;if(c>=1.2f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4+0.6f+d-3.4f-1.2f;leg[5]=leg5-0.6f+d-3.4f+1.2f*0.6f;
			leg[6]=leg6-0.6f-d+3.4f+1.2f;leg[7]=leg7+0.6f-d+3.4f-1.2f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=1.2f;i2>0;i2-=add2){
		leg[4]=leg4+0.6f-i2;leg[5]=leg5-0.6f+i2*0.6f;
		leg[6]=leg6-0.6f+i2;leg[7]=leg7+0.6f-i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0-0.6f+b2;leg[1]=leg1+0.6f+b2;
			leg[2]=leg2+0.6f-b2;leg[3]=leg3-0.6f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0-0.6f+3.4f+c2;leg[1]=leg1+0.6f+3.4f-c2*0.6f;
			leg[2]=leg2+0.6f-3.4f-c2;leg[3]=leg3-0.6f-3.4f+c2*0.6f;
			c2+=add;if(c2>=1.2f)c2=1.2f;if(c2>=1.2f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0-0.6f-d2+3.4f+1.2f;leg[1]=leg1+0.6f-d2+3.4f-1.2f*0.6f;
			leg[2]=leg2+0.6f+d2-3.4f-1.2f;leg[3]=leg3-0.6f+d2-3.4f+1.2f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_back_bit_still_out(void)
{
	for(fp32 i=0;i<=0.6f;i+=0.00005f){
	leg[0]=leg0+0.6f-i;leg[1]=leg1-0.6f*0.2f+i*0.2f;leg[2]=leg2-0.6f+i;leg[3]=leg3+0.6f*0.2f-i*0.2f;
	leg[4]=leg4+0.6f-i;leg[5]=leg5-0.6f+i;leg[6]=leg6-0.6f+i;leg[7]=leg7+0.6f-i;}
}

/**************************************微前进**************************************/
void dog_forward_bit_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
	leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_forward_bit(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0+1.2f-i*0.6f;leg[1]=leg1-1.2f+i;
		leg[2]=leg2-1.2f+i*0.6f;leg[3]=leg3+1.2f-i;
		if(flag_1==0){
			leg[4]=leg4-1.2f-b;leg[5]=leg5+1.2f-b;
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4-1.2f-3.4f+c*0.6f;leg[5]=leg5+1.2f-3.4f-c;
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4-1.2f+d-3.4f+2.4f*0.6f;leg[5]=leg5+1.2f+d-3.4f-2.4f;
			leg[6]=leg6+1.2f-d+3.4f-2.4f*0.6f;leg[7]=leg7-1.2f-d+3.4f+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4-1.2f+i2*0.6f;leg[5]=leg5+1.2f-i2;
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[0]=leg0+1.2f+b2;leg[1]=leg1-1.2f+b2;
			leg[2]=leg2-1.2f-b2;leg[3]=leg3+1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+1.2f+3.4f-c2*0.6f;leg[1]=leg1-1.2f+3.4f+c2;
			leg[2]=leg2-1.2f-3.4f+c2*0.6f;leg[3]=leg3+1.2f-3.4f-c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+1.2f-d2+3.4f-2.4f*0.6f;leg[1]=leg1-1.2f-d2+3.4f+2.4f;
			leg[2]=leg2-1.2f+d2-3.4f+2.4f*0.6f;leg[3]=leg3+1.2f+d2-3.4f-2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}

void dog_forward_bit_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0-1.2f*0.2f+i*0.2f;leg[1]=leg1+1.2f-i;leg[2]=leg2+1.2f*0.2f-i*0.2f;leg[3]=leg3-1.2f+i;
	leg[4]=leg4-1.2f+i;leg[5]=leg5+1.2f-i;leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f + i;}
}
void dog_forward_bit_dir(fp32 add, fp32 add2)
{
    fp32 comp_right = 1.0f, comp_left = 1.0f;
    const fp32 dz = 176.0f / 660.0f;
    float dev = (float)(RC_Ctl.rc.ch2 - 1024) / 660.0f;
    if (dev > dz) {
        float factor = (dev - dz) / (1.0f - dz);
        comp_right = 1.0f - factor * 0.80f;
        comp_left  = 1.0f + factor * 0.80f;
        if(comp_right < 0.20f) comp_right = 0.20f;
        if(comp_left  > 1.80f) comp_left  = 1.80f;
    } else if (dev < -dz) {
        float factor = (-dev - dz) / (1.0f - dz);
        comp_left  = 1.0f - factor * 0.80f;
        comp_right = 1.0f + factor * 0.80f;
        if(comp_left  < 0.20f) comp_left  = 0.20f;
        if(comp_right > 1.80f) comp_right = 1.80f;
    }
    flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
    for(fp32 i=2.4f;i>0;i-=add2){
        leg[0]=(1.2f-i*0.6f)*comp_right+leg0;  leg[1]=(-1.2f+i)*comp_right+leg1;
        leg[2]=(-1.2f+i*0.6f)*comp_left+leg2;  leg[3]=(1.2f-i)*comp_left+leg3;
        if(flag_1==0){
            leg[4]=(-1.2f-b)*comp_left+leg4;             leg[5]=(1.2f-b)*comp_left+leg5;
            leg[6]=(1.2f+b)*comp_right+leg6;             leg[7]=(-1.2f+b)*comp_right+leg7;
            b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
        if(flag_1==1){
            leg[4]=(-1.2f-3.4f+c*0.6f)*comp_left+leg4;  leg[5]=(1.2f-3.4f-c)*comp_left+leg5;
            leg[6]=(1.2f+3.4f-c*0.6f)*comp_right+leg6;  leg[7]=(-1.2f+3.4f+c)*comp_right+leg7;
            c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
        if(flag_1==2){
            leg[4]=(-1.2f+d-3.4f+2.4f*0.6f)*comp_left+leg4;  leg[5]=(1.2f+d-3.4f-2.4f)*comp_left+leg5;
            leg[6]=(1.2f-d+3.4f-2.4f*0.6f)*comp_right+leg6;  leg[7]=(-1.2f-d+3.4f+2.4f)*comp_right+leg7;
            d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
    for(fp32 i2=2.4f;i2>0;i2-=add2){
        leg[4]=(-1.2f+i2*0.6f)*comp_left+leg4;  leg[5]=(1.2f-i2)*comp_left+leg5;
        leg[6]=(1.2f-i2*0.6f)*comp_right+leg6;  leg[7]=(-1.2f+i2)*comp_right+leg7;
        if(flag_2==0){
            leg[0]=(1.2f+b2)*comp_right+leg0;            leg[1]=(-1.2f+b2)*comp_right+leg1;
            leg[2]=(-1.2f-b2)*comp_left+leg2;            leg[3]=(1.2f-b2)*comp_left+leg3;
            b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
        if(flag_2==1){
            leg[0]=(1.2f+3.4f-c2*0.6f)*comp_right+leg0;  leg[1]=(-1.2f+3.4f+c2)*comp_right+leg1;
            leg[2]=(-1.2f-3.4f+c2*0.6f)*comp_left+leg2;  leg[3]=(1.2f-3.4f-c2)*comp_left+leg3;
            c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
        if(flag_2==2){
            leg[0]=(1.2f-d2+3.4f-2.4f*0.6f)*comp_right+leg0;  leg[1]=(-1.2f-d2+3.4f+2.4f)*comp_right+leg1;
            leg[2]=(-1.2f+d2-3.4f+2.4f*0.6f)*comp_left+leg2;  leg[3]=(1.2f+d2-3.4f-2.4f)*comp_left+leg3;
            d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
/**************************************微左转**************************************/
void dog_left_bit_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){leg[0]=leg0-i*0.2f;leg[1]=leg1+i;leg[2]=leg2-i;
	leg[3]=leg3+i*0.2f;leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6+i;leg[7]=leg7-i;}
	kd = 0.5f;
}
void dog_left_bit(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0+1.2f-i*0.6f;leg[1]=leg1-1.2f+i;
		leg[2]=leg2+1.2f-i;leg[3]=leg3-1.2f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+1.2f-b;leg[5]=leg5-1.2f-b;
			leg[6]=leg6+1.2f+b;leg[7]=leg7-1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+1.2f-3.4f-c;leg[5]=leg5-1.2f-3.4f+c*0.6f;
			leg[6]=leg6+1.2f+3.4f-c*0.6f;leg[7]=leg7-1.2f+3.4f+c;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4+1.2f-3.4f+d-2.4f;leg[5]=leg5-1.2f-3.4f+d+2.4f*0.6f;
			leg[6]=leg6+1.2f+3.4f-d-2.4f*0.6f;leg[7]=leg7-1.2f+3.4f-d+2.4f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4+1.2f-i2;leg[5]=leg5-1.2f+i2*0.6f;
		leg[6]=leg6+1.2f-i2*0.6f;leg[7]=leg7-1.2f+i2;
		if(flag_2==0){
			leg[0]=leg0+1.2f+b2;leg[1]=leg1-1.2f+b2;
			leg[2]=leg2+1.2f-b2;leg[3]=leg3-1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0+1.2f+3.4f-c2*0.6f;leg[1]=leg1-1.2f+3.4f+c2;
			leg[2]=leg2+1.2f-3.4f-c2;leg[3]=leg3-1.2f-3.4f+c2*0.6f;
			c2+=add;if(c2>2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0+1.2f+3.4f-d2-2.4f*0.6f;leg[1]=leg1-1.2f+3.4f-d2+2.4f;
			leg[2]=leg2+1.2f-3.4f+d2-2.4f;leg[3]=leg3-1.2f-3.4f+d2+2.4f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_left_bit_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0-1.2f*0.2f+i*0.2f;leg[1]=leg1+1.2f-i;leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[4]=leg4+1.2f-i;leg[5]=leg5-1.2f+i;leg[6]=leg6+1.2f-i;leg[7]=leg7-1.2f+i;}
}
/**************************************微右转**************************************/
void dog_right_bit_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
		leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2+i*0.2f;leg[3]=leg3-i;
		leg[4]=leg4-i;leg[5]=leg5+i;leg[6]=leg6-i;leg[7]=leg7+i;}
		kd = 0.5f;
}
void dog_right_bit(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0 - 1.2f + i;
		leg[1]=leg1 + 1.2f - i*0.6f;
		leg[2]=leg2 - 1.2f + i*0.6f;
		leg[3]=leg3 + 1.2f - i;
		if(flag_1==0){
			leg[4]=leg4 - 1.2f - b;
			leg[5]=leg5 + 1.2f - b;
			leg[6]=leg6 - 1.2f + b;
			leg[7]=leg7 + 1.2f + b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4 - 1.2f - 3.4f + c*0.6f;
			leg[5]=leg5 + 1.2f - 3.4f - c;
			leg[6]=leg6 - 1.2f + 3.4f + c;
			leg[7]=leg7 + 1.2f + 3.4f - c*0.6f;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){	
			leg[4]=leg4 - 1.2f - 3.4f + d + 2.4f*0.6f;
			leg[5]=leg5 + 1.2f - 3.4f + d - 2.4f;
			leg[6]=leg6 - 1.2f + 3.4f - d + 2.4f;
			leg[7]=leg7 + 1.2f + 3.4f - d - 2.4f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4 - 1.2f + i2*0.6f;
		leg[5]=leg5 + 1.2f - i2;
		leg[6]=leg6 - 1.2f + i2;
		leg[7]=leg7 + 1.2f - i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0 - 1.2f + b2;
			leg[1]=leg1 + 1.2f + b2;
			leg[2]=leg2 - 1.2f - b2;
			leg[3]=leg3 + 1.2f - b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0 - 1.2f + 3.4f + c2;
			leg[1]=leg1 + 1.2f + 3.4f - c2*0.6f;
			leg[2]=leg2 - 1.2f - 3.4f + c2*0.6f;
			leg[3]=leg3 + 1.2f - 3.4f - c2;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0 - 1.2f + 3.4f - d2 + 2.4f;
			leg[1]=leg1 + 1.2f + 3.4f - d2 - 2.4f*0.6f;
			leg[2]=leg2 - 1.2f - 3.4f + d2 + 2.4f*0.6f;
			leg[3]=leg3 + 1.2f - 3.4f + d2 - 2.4f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_right_bit_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+1.2f-i;leg[1]=leg1-1.2f*0.2f+i*0.2f;leg[2]=leg2+1.2f*0.2f-i*0.2f;leg[3]=leg3-1.2f+i;
	leg[4]=leg4-1.2f+i;leg[5]=leg5+1.2f-i;leg[6]=leg6-1.2f+i;leg[7]=leg7+1.2f-i;}
}
/**************************************微后退**************************************/
void dog_back_bit_int(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+i;leg[1]=leg1-i*0.2f;leg[2]=leg2-i;leg[3]=leg3+i*0.2f;
	leg[4]=leg4+i;leg[5]=leg5-i;leg[6]=leg6-i;leg[7]=leg7+i;}
	kd = 0.5f;
}
void dog_back_bit(fp32 add,fp32 add2)
{
	flag_1=0;flag_2=0;b=0,b2=0;c=0,c2=0;d=0,d2=0;
	for(fp32 i=2.4f;i>0;i-=add2){
		leg[0]=leg0-1.2f+i;leg[1]=leg1+1.2f-i*0.6f;
		leg[2]=leg2+1.2f-i;leg[3]=leg3-1.2f+i*0.6f;
		if(flag_1==0){
			leg[4]=leg4+1.2f-b;leg[5]=leg5-1.2f-b;
			leg[6]=leg6-1.2f+b;leg[7]=leg7+1.2f+b;
			b+=add;if(b>=3.4f)b=3.4f;if(b>=3.4f)flag_1=1;}
		if(flag_1==1){
			leg[4]=leg4+1.2f-3.4f-c;leg[5]=leg5-1.2f-3.4f+c*0.6f;
			leg[6]=leg6-1.2f+3.4f+c;leg[7]=leg7+1.2f+3.4f-c*0.6f;
			c+=add;if(c>=2.4f)c=2.4f;if(c>=2.4f)flag_1=2;}
		if(flag_1==2){
			leg[4]=leg4+1.2f+d-3.4f-2.4f;leg[5]=leg5-1.2f+d-3.4f+2.4f*0.6f;
			leg[6]=leg6-1.2f-d+3.4f+2.4f;leg[7]=leg7+1.2f-d+3.4f-2.4f*0.6f;
			d+=add;if(d>=3.4f)d=3.4f;if(d>=3.4f)flag_1=4;}}
	for(fp32 i2=2.4f;i2>0;i2-=add2){
		leg[4]=leg4+1.2f-i2;leg[5]=leg5-1.2f+i2*0.6f;
		leg[6]=leg6-1.2f+i2;leg[7]=leg7+1.2f-i2*0.6f;
		if(flag_2==0){
			leg[0]=leg0-1.2f+b2;leg[1]=leg1+1.2f+b2;
			leg[2]=leg2+1.2f-b2;leg[3]=leg3-1.2f-b2;
			b2+=add;if(b2>=3.4f)b2=3.4f;if(b2>=3.4f)flag_2=1;}
		if(flag_2==1){
			leg[0]=leg0-1.2f+3.4f+c2;leg[1]=leg1+1.2f+3.4f-c2*0.6f;
			leg[2]=leg2+1.2f-3.4f-c2;leg[3]=leg3-1.2f-3.4f+c2*0.6f;
			c2+=add;if(c2>=2.4f)c2=2.4f;if(c2>=2.4f)flag_2=2;}
		if(flag_2==2){
			leg[0]=leg0-1.2f-d2+3.4f+2.4f;leg[1]=leg1+1.2f-d2+3.4f-2.4f*0.6f;
			leg[2]=leg2+1.2f+d2-3.4f-2.4f;leg[3]=leg3-1.2f+d2-3.4f+2.4f*0.6f;
			d2+=add;if(d2>=3.4f)d2=3.4f;if(d2>=3.4f)flag_2=4;}}
}
void dog_back_bit_out(void)
{
	for(fp32 i=0;i<=1.2f;i+=0.00005f){
	leg[0]=leg0+1.2f-i;leg[1]=leg1-1.2f*0.2f+i*0.2f;leg[2]=leg2-1.2f+i;leg[3]=leg3+1.2f*0.2f-i*0.2f;
	leg[4]=leg4+1.2f-i;leg[5]=leg5-1.2f+i;leg[6]=leg6-1.2f+i;leg[7]=leg7+1.2f-i;}
}


/**************************************上台阶*************************																																																																																																																																																																																																																					*************///没用到
void dog_stair_int(void)
{
//  for(fp32 i=0;i<=8.4f;i+=0.00006f)
//	{
//			leg0=leg_middle[0]+3.2-i;leg1=leg_middle[1]+3.2-i;leg2=leg_middle[2]-3.2+i;leg3=leg_middle[3]-3.2+i;
//	    leg4=leg_middle[4]-3.2+i;leg5=leg_middle[5]-3.2+i;leg6=leg_middle[6]+3.2-i;leg7=leg_middle[7]+3.2-i;
//		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
//	}
//  
//	for(fp32 i=0;i<=2.2f;i+=0.00006f)
//	{
//			leg0=leg_middle[0]+3.2-8.4+i;leg1=leg_middle[1]+3.2-8.4-i;leg2=leg_middle[2]-3.2+8.4-i;leg3=leg_middle[3]-3.2+8.4+i;
//	    leg4=leg_middle[4]-3.2+8.4-i;leg5=leg_middle[5]-3.2+8.4+i;leg6=leg_middle[6]+3.2-8.4+i;leg7=leg_middle[7]+3.2-8.4-i;
//		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
//	}
  for(fp32 i=0;i<=10.6f;i+=0.00006f)
	{
			leg0=leg_middle[0]+3.2-i*0.5849f;leg1=leg_middle[1]+3.2-i;leg2=leg_middle[2]-3.2+i*0.5849f;leg3=leg_middle[3]-3.2+i;
	    leg4=leg_middle[4]-3.2+i*0.5849f;leg5=leg_middle[5]-3.2+i;leg6=leg_middle[6]+3.2-i*0.5849f;leg7=leg_middle[7]+3.2-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	kd = 0.8f;
}

void dog_stair_int_still(void)
{
			leg0=leg_middle[0]+3.2-8.4+2.2;leg1=leg_middle[1]+3.2-8.4-2.2;leg2=leg_middle[2]-3.2+8.4-2.2;leg3=leg_middle[3]-3.2+8.4+2.2;
	    leg4=leg_middle[4]-3.2+8.4-2.2;leg5=leg_middle[5]-3.2+8.4+2.2;leg6=leg_middle[6]+3.2-8.4+2.2;leg7=leg_middle[7]+3.2-8.4-2.2;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	    kd = 0.8f;
}
void dog_stair(void)//史山
{
	for(fp32 i=0;i<=11.4f;i+=0.0005f)//收
	{
			//leg0=leg_middle[0]+3.2-8.4+2.2+i;leg1=leg_middle[1]+3.2-8.4-2.2+i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-i;leg3=leg_middle[3]-3.2+8.4+2.2-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=7.1f;i+=0.0005f)//移
	{
			//leg0=leg_middle[0]+3.2-8.4+2.2+6.4-i;leg1=leg_middle[1]+3.2-8.4-2.2+6.4+i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-6.4+i;leg3=leg_middle[3]-3.2+8.4+2.2-6.4-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=11.4f;i+=0.0005f)//出
	{
		
			//leg0=leg_middle[0]+3.2-8.4+2.2+6.4-7.1-i;leg1=leg_middle[1]+3.2-8.4-2.2+6.4+7.1-i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-9.4+7.1+i;leg3=leg_middle[3]-3.2+8.4+2.2-9.4-7.1+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  for(fp32 i=0;i<=11.4f;i+=0.0005f)//收
	{
		  leg6=leg_middle[6]+3.2-8.4+2.2+i;leg7=leg_middle[7]+3.2-8.4-2.2+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=7.1f;i+=0.0005f)//移
	{
		  leg6=leg_middle[6]+3.2-8.4+2.2+6.4-i;leg7=leg_middle[7]+3.2-8.4-2.2+6.4+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=11.4f;i+=0.0005f)//出
	{
		  leg6=leg_middle[6]+3.2-8.4+2.2+9.4-7.1-i;leg7=leg_middle[7]+3.2-8.4-2.2+9.4+7.1-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=6.4f;i+=0.0005f)//一起收
	{
		  leg2=leg_middle[2]-3.2+8.4-2.2+7.1-i;leg3=leg_middle[3]-3.2+8.4+2.2-7.1-i;
		  leg6=leg_middle[6]+3.2-8.4+2.2-7.1+i;leg7=leg_middle[7]+3.2-8.4-2.2+7.1+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=3.2f;i+=0.0001f)//一起出
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2-i;leg1=leg_middle[1]+3.2-8.4-2.2-i;
	    leg4=leg_middle[4]-3.2+8.4-2.2+i;leg5=leg_middle[5]-3.2+8.4+2.2+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=7.1f;i+=0.0003f)//一起移
	{
		  leg2=leg_middle[2]-3.2+8.4-2.2+7.1-6.4-i;leg3=leg_middle[3]-3.2+8.4+2.2-7.1-6.4+i;
		  leg6=leg_middle[6]+3.2-8.4+2.2-7.1+6.4+i;leg7=leg_middle[7]+3.2-8.4-2.2+7.1+6.4-i;
		  leg0=leg_middle[0]+3.2-8.4+2.2-3.2+i*0.5f;leg1=leg_middle[1]+3.2-8.4-2.2-3.2-i*0.5f;
	    leg4=leg_middle[4]-3.2+8.4-2.2+3.2-i*0.5f;leg5=leg_middle[5]-3.2+8.4+2.2+3.2+i*0.5f;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=13.8f;i+=0.0005f)//收
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2-3.2+7.1*0.5+i;leg1=leg_middle[1]+3.2-8.4-2.2-3.2-7.1*0.5+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=4.4f;i+=0.0005f)//移
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2-3.2+7.1*0.5+13.8-i;leg1=leg_middle[1]+3.2-8.4-2.2-3.2-7.1*0.5+13.8+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=2.8f;i+=0.0005f)//出
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2-3.2+7.1*0.5+13.8-4.4-i;leg1=leg_middle[1]+3.2-8.4-2.2-3.2-7.1*0.5+13.8+4.4-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}		  //leg0=leg_middle[0]+3.2-8.4+2.2-3.2+7.1*0.5+8.8-4.1-2.8;leg1=leg_middle[1]+3.2-8.4-2.2-3.2-7.1*0.5+8.8+4.1-2.8;
	for(fp32 i=0;i<=13.8f;i+=0.0005f)//收
	{
	    leg4=leg_middle[4]-3.2+8.4-2.2+3.2-7.1*0.5-i;leg5=leg_middle[5]-3.2+8.4+2.2+3.2+7.1*0.5-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=4.4f;i+=0.0005f)//移
	{
	    leg4=leg_middle[4]-3.2+8.4-2.2+3.2-7.1*0.5-13.8+i;leg5=leg_middle[5]-3.2+8.4+2.2+3.2+7.1*0.5-13.8-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=2.8f;i+=0.0005f)//出
	{
	    leg4=leg_middle[4]-3.2+8.4-2.2+3.2-7.1*0.5-13.8+4.4+i;leg5=leg_middle[5]-3.2+8.4+2.2+3.2+7.1*0.5-13.8-4.4+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  for(fp32 i=0;i<=0.85f;i+=0.0005f)//
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2-0.85+7.8+i;leg1=leg_middle[1]+3.2-8.4-2.2+0.85+7.8-i;
	    leg4=leg_middle[4]-3.2+8.4-2.2+0.85-7.8-i;leg5=leg_middle[5]-3.2+8.4+2.2-0.85-7.8+i;		  
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=7.8f;i+=0.0005f)//
	{
		  leg0=leg_middle[0]+3.2-8.4+2.2+7.8-i;leg1=leg_middle[1]+3.2-8.4-2.2+7.8-i;
	    leg4=leg_middle[4]-3.2+8.4-2.2-7.8+i;leg5=leg_middle[5]-3.2+8.4+2.2-7.8+i;
	    leg2=leg_middle[2]-3.2+8.4-2.2-6.4+i*0.82f;leg3=leg_middle[3]-3.2+8.4+2.2-6.4+i*0.82f;
		  leg6=leg_middle[6]+3.2-8.4+2.2+6.4-i*0.82f;leg7=leg_middle[7]+3.2-8.4-2.2+6.4-i*0.82f;
		  
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	kd = 0.8f;
}	
void dog_stair_out(void)
{
//  for(fp32 i=0;i<=2.2f;i+=0.00006f)
//	{
//			leg0=leg_middle[0]+3.2-8.4+2.2-i;leg1=leg_middle[1]+3.2-8.4-2.2+i;leg2=leg_middle[2]-3.2+8.4-2.2+i;leg3=leg_middle[3]-3.2+8.4+2.2-i;
//	    leg4=leg_middle[4]-3.2+8.4-2.2+i;leg5=leg_middle[5]-3.2+8.4+2.2-i;leg6=leg_middle[6]+3.2-8.4+2.2-i;leg7=leg_middle[7]+3.2-8.4-2.2+i;
//		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
//	}
//	for(fp32 i=0;i<=8.4f;i+=0.00006f)
//	{
//			leg0=leg_middle[0]+3.2-8.4+i;leg1=leg_middle[1]+3.2-8.4+i;leg2=leg_middle[2]-3.2+8.4-i;leg3=leg_middle[3]-3.2+8.4-i;
//	    leg4=leg_middle[4]-3.2+8.4-i;leg5=leg_middle[5]-3.2+8.4-i;leg6=leg_middle[6]+3.2-8.4+i;leg7=leg_middle[7]+3.2-8.4+i;
//		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
//			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
//	}
	  for(fp32 i=0;i<=10.6f;i+=0.00006f)
		{
				leg0=leg_middle[0]+3.2-6.2+i*0.5849f;leg1=leg_middle[1]+3.2-8.4-2.2+i;leg2=leg_middle[2]-3.2+6.2-i*0.5849f;leg3=leg_middle[3]-3.2+8.4+2.2-i;
				leg4=leg_middle[4]-3.2+6.2-i*0.5849f;leg5=leg_middle[5]-3.2+8.4+2.2-i;leg6=leg_middle[6]+3.2-6.2+i*0.5849f;leg7=leg_middle[7]+3.2-8.4-2.2+i;
				leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
				leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
		}
	kd = 0.5f;
}
/**************************************升高*************************																																																																																																																																																																																																																					*************///没用到
void dog_n2high(void)
{
	for(fp32 i=0;i<=0.8f;i+=0.00012f)
	{
			leg0=leg_middle[0]-3.2-i;leg1=leg_middle[1]-3.2-i;leg2=leg_middle[2]+3.2+i;leg3=leg_middle[3]+3.2+i;
	    leg4=leg_middle[4]+3.2+i;leg5=leg_middle[5]+3.2+i;leg6=leg_middle[6]-3.2-i;leg7=leg_middle[7]-3.2-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  kd = 0.5f;
}

void dog_high_still(void)
{	
			leg0=leg_middle[0]-3.2-0.8;leg1=leg_middle[1]-3.2-0.8;leg2=leg_middle[2]+3.2+0.8;leg3=leg_middle[3]+3.2+0.8;
	    leg4=leg_middle[4]+3.2+0.8;leg5=leg_middle[5]+3.2+0.8;leg6=leg_middle[6]-3.2-0.8;leg7=leg_middle[7]-3.2-0.8;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
      kd = 0.5f;
}

void dog_high2n(void)
{	
	for(fp32 i=0;i<=0.8f;i+=0.00012f)
	{
			leg0=leg_middle[0]-4.0+i;leg1=leg_middle[1]-4.0+i;leg2=leg_middle[2]+4.0-i;leg3=leg_middle[3]+4.0-i;
	    leg4=leg_middle[4]+4.0-i;leg5=leg_middle[5]+4.0-i;leg6=leg_middle[6]-4.0+i;leg7=leg_middle[7]-4.0+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  kd = 0.5f;
}


void dog_high2squat(void)
{	
	for(fp32 i=0;i<=7.2f;i+=0.00024f)
	{
			leg0=leg_middle[0]-4.0+i;leg1=leg_middle[1]-4.0+i;leg2=leg_middle[2]+4.0-i;leg3=leg_middle[3]+4.0-i;
	    leg4=leg_middle[4]+4.0-i;leg5=leg_middle[5]+4.0-i;leg6=leg_middle[6]-4.0+i;leg7=leg_middle[7]-4.0+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  kd = 0.5f;
}

void dog_squat2high(void)
{	
	for(fp32 i=0;i<=7.2f;i+=0.00024f)
	{
			leg0=leg_middle[0]+3.2-7.2;leg1=leg_middle[1]+3.2-7.2;leg2=leg_middle[2]-3.2+7.2;leg3=leg_middle[3]-3.2+7.2;
	    leg4=leg_middle[4]-3.2+7.2;leg5=leg_middle[5]-3.2+7.2;leg6=leg_middle[6]+3.2-7.2;leg7=leg_middle[7]+3.2-7.2;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
  kd = 0.5f;
}

/**************************************木桥b*************************																																																																																																																																																																																																																					*************///没用到
void dog_bridge(void)
{	
	for(fp32 i=0;i<=3.5f;i+=0.00048f)//收
	{
			leg0=leg_middle[0]+3.2-8.4+2.2+i;leg1=leg_middle[1]+3.2-8.4-2.2+i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-i;leg3=leg_middle[3]-3.2+8.4+2.2-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=4.4f;i+=0.00048f)//向前移
	{
			leg0=leg_middle[0]+3.2-8.4+2.2+3.5-i;leg1=leg_middle[1]+3.2-8.4-2.2+3.5+i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-3.5+i;leg3=leg_middle[3]-3.2+8.4+2.2-3.5-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=3.5f;i+=0.00048f)//伸
	{
			leg0=leg_middle[0]+3.2-8.4+2.2+3.5-4.4-i;leg1=leg_middle[1]+3.2-8.4-2.2+3.5+4.4-i;
		  leg2=leg_middle[2]-3.2+8.4-2.2-3.5+4.4+i;leg3=leg_middle[3]-3.2+8.4+2.2-3.5-4.4+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=3.5f;i+=0.00048f)//收
	{
			leg4=leg_middle[4]-3.2+8.4-2.2-i;leg5=leg_middle[5]-3.2+8.4+2.2-i;
		  leg6=leg_middle[6]+3.2-8.4+2.2+i;leg7=leg_middle[7]+3.2-8.4-2.2+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=4.4f;i+=0.00048f)//向前移
	{
			leg4=leg_middle[4]-3.2+8.4-2.2-3.5+i;leg5=leg_middle[5]-3.2+8.4+2.2-3.5-i;
		  leg6=leg_middle[6]+3.2-8.4+2.2+3.5-i;leg7=leg_middle[7]+3.2-8.4-2.2+3.5+i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=3.5f;i+=0.00048f)//伸
	{
		  leg4=leg_middle[4]-3.2+8.4-2.2-3.5+4.4+i;leg5=leg_middle[5]-3.2+8.4+2.2-3.5-4.4+i;
		  leg6=leg_middle[6]+3.2-8.4+2.2+3.5-4.4-i;leg7=leg_middle[7]+3.2-8.4-2.2+3.5+4.4-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	for(fp32 i=0;i<=4.4f;i+=0.00048f)//向前移
	{
			leg0=leg_middle[0]+3.2-8.4+2.2-4.4+i;leg1=leg_middle[1]+3.2-8.4-2.2+4.4-i;
		  leg2=leg_middle[2]-3.2+8.4-2.2+4.4-i;leg3=leg_middle[3]-3.2+8.4+2.2-4.4+i;
		  leg4=leg_middle[4]-3.2+8.4-2.2+4.4-i;leg5=leg_middle[5]-3.2+8.4+2.2-4.4+i;
		  leg6=leg_middle[6]+3.2-8.4+2.2-4.4+i;leg7=leg_middle[7]+3.2-8.4-2.2+4.4-i;
		  leg[0]=leg0;leg[1]=leg1;leg[2]=leg2;leg[3]=leg3;
			leg[4]=leg4;leg[5]=leg5;leg[6]=leg6;leg[7]=leg7;
	}
	
	kd = 0.8f;
}
