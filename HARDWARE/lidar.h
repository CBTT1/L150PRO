/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-07-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-07-05

All rights reserved
***********************************************/

#ifndef __LIDAR_H
#define	__LIDAR_H

#include "system.h"



//#define Lidar_Follow_Mode					2
//#define Lidar_Avoid_Mode					1

#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define TAIL_0  0x3B
#define TAIL_1  0xB3
#define Length_ 0x6C

#define POINT_PER_PACK 32






typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

// 单辆小车的数据结构
typedef struct CarSpeedData
{
	uint8_t car_id;    // 小车ID (0-9)
	uint8_t dir_x;     // 00为正 01为负
	uint8_t speed_x;
	uint8_t dir_y;
	uint8_t speed_y;
	uint8_t dir_z;
	uint8_t speed_z;
}CarSpeedDataDef;

// 数据包结构
typedef struct PackSpeedData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t car_count;  // 有速度的小车数量 (0-5)
	CarSpeedDataDef current_car;  // 当前小车的数据
	uint8_t tail_0;
	uint8_t tail_1;
	uint8_t crc_num;
}SpeedFrameTypeDef;

typedef struct PointDataProcess_
{
	uint16_t distance;
	float angle;
}PointDataProcessDef;

typedef struct SpeedDataProcess_
{
	uint8_t dir_x; // 00为正 01为负
	uint8_t speed_x;
	uint8_t dir_y;
	uint8_t speed_y;
	uint8_t dir_z;
	uint8_t speed_z;
}SpeedDataProcessDef;

extern PointDataProcessDef PointDataProcess[1200];//更新225个数据
extern LiDARFrameTypeDef Pack_Data;
extern SpeedFrameTypeDef Pack_SpeedData; // 一个包含所有小车数据的包
extern PointDataProcessDef Dataprocess[1200];//用于小车避障、跟随、走直线、ELE雷达避障的雷达数据
extern SpeedDataProcessDef SpeedDataProcess;
extern u8 CarCount;

void LIDAR_USART_Init(void);
void UART5_IRQHandler(void);
void data_process(uint8_t reset);

extern float Distance_KP,Distance_KD,Distance_KI;		//距离调整PID参数
extern float Follow_KP,Follow_KD,Follow_KI;  //转向PID
extern float Follow_KP_Akm,Follow_KD_Akm,Follow_KI_Akm;

extern float Diff_Along_Distance_KP,Diff_Along_Distance_KD,Diff_Along_Distance_KI;	//距离调整PID参数
extern float Akm_Along_Distance_KP,Akm_Along_Distance_KD,Akm_Along_Distance_KI;	//距离调整PID参数
extern float FourWheel_Along_Distance_KP,FourWheel_Along_Distance_KD,FourWheel_Along_Distance_KI;	//距离调整PID参数
extern float Along_Distance_KP,Along_Distance_KD,Along_Distance_KI;		//距离调整PID参数

float Distance_Adjust_PID(float Current_Distance,float Target_Distance);
void Get_Target_Encoder(float Vx,float Vz);
int Incremental_PI_Left (int Encoder,int Target);
int Incremental_PI_Right (int Encoder,int Target);
float Follow_Turn_PID(float Current_Angle,float Target_Angle);
float Along_Adjust_PID(float Current_Distance,float Target_Distance);
#endif

