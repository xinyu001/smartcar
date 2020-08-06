#ifndef __FUNCTION_H__
#define __FUNCTION_H__

void init();
void Para_Init();
float Slope_Calculate(uint8 begin,uint8 end,float *p);

extern int   AD_val_1;
extern int   AD_val_2;
extern int   AD_val_3;
extern int   adtmp1,adtmp2,adtmp3;
extern int   dis_AD_val_1,dis_AD_val_2,dis_AD_val_3 ;
extern int   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3 ;

extern int   AD_val_1_min;
extern int   AD_val_2_min;
extern int   AD_val_3_min;

extern int   AD_val_1_max;
extern int   AD_val_2_max;
extern int   AD_val_3_max; 

extern unsigned char  Dir_last;
extern int  dir_error_pre;
extern int  dir_error;
extern int dis_error;
extern unsigned char zz;


#endif