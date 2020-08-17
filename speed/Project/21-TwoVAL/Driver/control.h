#ifndef _CONTROL_H_
#define _CONTROL_H_
typedef struct PID{float P,pout,I,iout,D,dout,OUT;}PID;
extern  PID PID_SPEED,PID_TURN;

//电磁相关
extern float   AD_val_1_min;
extern float   AD_val_2_min;
extern float   AD_val_3_min;
extern float   AD_val_1_max;
extern float   AD_val_2_max;
extern float   AD_val_3_max; 
extern float AD_val_1,AD_val_2,AD_val_3,AD_val_4;
extern float dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4;
extern float disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4;
extern int circle_Flag;
extern int turn_Flag;
extern int turn_Flag2;
extern int turn_right_Flag;
extern int turn_left_Flag;
extern int Go_Out_Circle;
extern int   Stop_Brake;                               //刹车
extern int   NitroBooster;
extern float acceleration;

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
//模糊化相关
extern float  Delta_P;
extern float  Delta_D;
extern float  Fuzzy_Kp;
extern float  Fuzzy_Kd;
//方向控制相关
extern int    DirectionCount;
extern float  Delt_error,Middle_Err;
extern float  Turn_Speed,Turn_Out,Turn_Angle_Integral;


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
