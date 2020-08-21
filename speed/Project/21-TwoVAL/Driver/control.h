#ifndef _CONTROL_H_
#define _CONTROL_H_
typedef struct PID{float P,pout,I,iout,D,dout,OUT;}PID;
extern  PID PID_SPEED,PID_TURN;
extern  float Kp,Ki;

extern uint8 Style;
extern int Speed_Filter_Times;
extern int SpeedCount;
extern float CarSpeed,ControlSpeed,Hill_Slow_Ratio;
extern float SetSpeed;
extern float Speed_H,Speed_M,Speed_L;
extern uint8 Set_Angle;
extern float AverageSpeed;
extern float Distance;
extern float SpeedControlOutOld,SpeedControlOutNew;
extern float SpeedControlIntegral;
extern float MotorOut;
extern int Stop_Brake;
extern float acceleration;
extern int   NitroBooster;
//模糊化相关
extern float  Delta_P;
extern float  Delta_D;
extern float  Fuzzy_Kp;
extern float  Fuzzy_Kd;
//方向控制相关
extern int    DirectionCount;
extern float  Delt_error,Middle_Err;
extern float  Turn_Speed,Turn_Out,Turn_Angle_Integral;
extern int circle_Flag;
extern int Go_Out_Circle;
extern int flag_cricle_right,flag_cricle_left;         //电磁感应环岛
//extern float AD_val_1,AD_val_2,AD_val_3,AD_val_4,AD_val_5,AD_val_6;
//extern float dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4,dis_AD_val_5,dis_AD_val_6;
//extern float disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4,disgy_AD_val_5,disgy_AD_val_6;
//extern float AD_val_1_max;
//extern float AD_val_2_max;
//extern float AD_val_3_max;
//extern float AD_val_4_max;         //大概两边相等放中间归一化是40几
//extern float AD_val_5_max;
//extern float AD_val_6_max;
//extern float AD_val_1_min;
//extern float AD_val_2_min;
//extern float AD_val_3_min;
//extern float AD_val_4_min;
//extern float AD_val_5_min;
//extern float AD_val_6_min;

/**舵机相关**/
extern int sever_middle;
extern int sever_range;

extern uint8 Starting,Stop;

void Get_Attitude();
void Get_Speed();
void Strong_Turn();
void Moto_Out();
void Speed_Control();
void Speed_Control_Output();
void Direction_Control();

float  Middle_Err_Filter(float middle_err);  
float  Turn_Out_Filter(float turn_out);
#endif
