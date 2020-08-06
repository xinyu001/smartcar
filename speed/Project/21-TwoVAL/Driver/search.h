#ifndef __SEARCH_H__
#define __SEARCH_H__
extern uint8  RoadType;
extern int vol0,Display1,Display2,Display3,flag;
extern int Display4,Display5,Display6;
extern float SlopeRight[6];
extern float Previous_Error[10];
extern float SlopeLeft[6];
extern float SlopeRight[6];
void Search();
void get_edge();   
//void sendimg();
#endif