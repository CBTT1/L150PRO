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

#include "lidar.h"
#include <string.h>
#include "sys.h"

float Diff_Along_Distance_KP = -0.030f,Diff_Along_Distance_KD = -0.245f,Diff_Along_Distance_KI = -0.001f;	//差速小车走直线距离调整PID参数
float Akm_Along_Distance_KP = -0.115f*1000,Akm_Along_Distance_KD = -1000.245f*1000,Akm_Along_Distance_KI = -0.001f*1000;	//阿克曼走直线距离调整PID参数
float FourWheel_Along_Distance_KP = -0.115f*1000,FourWheel_Along_Distance_KD = -100.200f*1000,FourWheel_Along_Distance_KI = -0.001f*1000;	//四驱走直线距离调整PID参数
float Along_Distance_KP = 0.163f*1000,Along_Distance_KD = 0.123*1000,Along_Distance_KI = 0.001f*1000;	//麦轮和全向走直线距离调整PID参数

float Distance_KP = -0.150f,Distance_KD = -0.052f,Distance_KI = -0.001;	//跟随距离调整PID参数

float Follow_KP = -2.566f,Follow_KD = -1.368f,Follow_KI = -0.001f;  //转向PID
float Follow_KP_Akm = -0.550f,Follow_KD_Akm = -0.121f,Follow_KI_Akm = -0.001f;
//float Distance_KP = 0.653f*1000,Distance_KD = 2.431f*1000,Distance_KI = 0.001*1000;	//距离调整PID参数

PointDataProcessDef PointDataProcess[1200];//更新225个数据
LiDARFrameTypeDef Pack_Data;
SpeedFrameTypeDef Pack_SpeedData; // 一个包含所有小车数据的包
PointDataProcessDef Dataprocess[1200];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据
SpeedDataProcessDef SpeedDataProcess; //支持最多10辆小车的速度数据
u8 CarCount = 0; //当前数据包中的小车数量


/**************************************************************************
Function: LIDAR_USART_Init
Input   : none
Output  : none
函数功能：雷达串口初始化
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//串口5

void LIDAR_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	USART_DeInit(UART5);  //复位串口5

	// 打开串口GPIO的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOC D时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟
	 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

	// 将USART Rx的GPIO配置为浮空输入模式
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化

	//配置USART为中断源 
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//抢断优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);


	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = 115200;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(UART5, &USART_InitStructure);

	// 使能串口接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	


	// 使能串口
	USART_Cmd(UART5, ENABLE);	    

	memset(&Pack_SpeedData, 0, sizeof(Pack_SpeedData));  // 将整个结构体初始化为0
	memset(&SpeedDataProcess, 0, sizeof(SpeedDataProcess));
}

///**************************************************************************
//Function: data_process
//Input   : none
//Output  : none
//函数功能：数据处理函数
//入口参数：无
//返回  值：无
//**************************************************************************/
////完成一帧接收后进行处理
//int data_cnt = 0;
//void data_process(void) //数据处理
//{
//	int m;
//	u32 distance_sum[32]={0};//2个点的距离和的数组
//	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//计算32个点的开始角度
//	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//计算32个点的结束角度
//	float area_angle[32]={0};
//	
//	if(start_angle>end_angle)//结束角度和开始角度被0度分割的情况
//		end_angle +=360;

//	for(m=0;m<32;m++)
//	{
//		area_angle[m]=start_angle+(end_angle-start_angle)/32*m;
//		if(area_angle[m]>360)  area_angle[m] -=360;
//		
//		distance_sum[m] +=((u16)Pack_Data.point[m].distance_h<<8)+Pack_Data.point[m].distance_l;//数据高低8位合并

//		Dataprocess[data_cnt].angle=area_angle[m];
//		Dataprocess[data_cnt++].distance=distance_sum[m];  //一帧数据为32个点
//		if(data_cnt == 1152) data_cnt = 0;
//	}
//	
//		
//}

/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
//完成一帧接收后进行处理
int data_cnt = 0;
void data_process(uint8_t reset) //数据处理
{
	if(reset == 1)
	{
		SpeedDataProcess.dir_x = 0;
		SpeedDataProcess.speed_x = 0;
		SpeedDataProcess.dir_y = 0;
		SpeedDataProcess.speed_y = 0;
		SpeedDataProcess.dir_z = 0;
		SpeedDataProcess.speed_z = 0;
	}
	else
	{
		SpeedDataProcess.dir_x = Pack_SpeedData.current_car.dir_x;
		SpeedDataProcess.speed_x = Pack_SpeedData.current_car.speed_x;
		SpeedDataProcess.dir_y = Pack_SpeedData.current_car.dir_y;
		SpeedDataProcess.speed_y = Pack_SpeedData.current_car.speed_y;
		SpeedDataProcess.dir_z = Pack_SpeedData.current_car.dir_z;
		SpeedDataProcess.speed_z = Pack_SpeedData.current_car.speed_z;		
	}

}


/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
void UART5_IRQHandler(void)
{	
	static u8 state = 0;//状态位	
	u8 temp_data;
	u8 reset = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //接收到数据
	{	
		temp_data=USART_ReceiveData(UART5);	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//头固定
				{
					Pack_SpeedData.header_0 = temp_data;
					state++;
				} else state = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//头固定
				{
					Pack_SpeedData.header_1 = temp_data;
					state++;
				} else state = 0;
				break;
			case 2:
				Pack_SpeedData.car_count = temp_data & 0x0F; //获取有速度的小车数量 (0-5)
				if(Pack_SpeedData.car_count > 5) Pack_SpeedData.car_count = 5; //限制最大数量为5
				if(Pack_SpeedData.car_count == 0) {
					state = 38; // 如果没有小车数据，直接跳转到包尾处理
				} else {
					state = 3; // 开始处理第一段数据
				}
				break;
			case 3: case 10: case 17: case 24: case 31:
				// 获取小车ID
				if((temp_data & 0x0F) == Car_Id) {
					Pack_SpeedData.current_car.car_id = temp_data & 0x0F;
				}
				else {
					Pack_SpeedData.current_car.car_id = 0;
				}
				state++;
				break;
			case 4: case 11: case 18: case 25: case 32:
				// dir_y
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.dir_y = temp_data;
				}
				state++;
				break;
			case 5: case 12: case 19: case 26: case 33:
				// speed_y
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.speed_y = temp_data;
				}
				state++;
				break;
			case 6: case 13: case 20: case 27: case 34:
				// dir_x
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.dir_x = temp_data;
				}
				state++;
				break;
			case 7: case 14: case 21: case 28: case 35:
				// speed_x
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.speed_x = temp_data;
				}
				state++;
				break;
			case 8: case 15: case 22: case 29: case 36:
				// dir_z
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.dir_z = temp_data;
				}
				state++;
				break;
			case 9: case 16: case 23: case 30: case 37:
				// speed_z
				if(Pack_SpeedData.current_car.car_id == Car_Id) {
					Pack_SpeedData.current_car.speed_z = temp_data;
				}
				state++;
				// 检查是否需要处理下一段数据
				if(state == 10 && Pack_SpeedData.car_count > 1) {
					state = 10; // 处理第二段数据
				}
				else if(state == 17 && Pack_SpeedData.car_count > 2) {
					state = 17; // 处理第三段数据
				}
				else if(state == 24 && Pack_SpeedData.car_count > 3) {
					state = 24; // 处理第四段数据
				}
				else if(state == 31 && Pack_SpeedData.car_count > 4) {
					state = 31; // 处理第五段数据
				}
				else {
					state = 38; // 所有数据处理完毕，跳转到包尾处理
				}
				break;
			case 38:
				if(temp_data == TAIL_0)//尾固定
				{
					Pack_SpeedData.tail_0 = temp_data;
					state++;
				} else state = 0;
				break;			
			case 39:
				if(temp_data == TAIL_1 && Pack_SpeedData.car_count != 0)//尾固定
				{
					Pack_SpeedData.tail_1 = temp_data;
				} 
				else
				{
					memset(&Pack_SpeedData, 0, sizeof(Pack_SpeedData));//清零
					reset = 1;
				}
				data_process(reset);
				state = 0;
				break;	
			default: break;
		}
	}		
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：走直线雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//走直线雷达距离调整pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == FourWheel_Car)
		OutPut=FourWheel_Along_Distance_KP*Bias/100000+FourWheel_Along_Distance_KI*Integral_bias/100000+FourWheel_Along_Distance_KD*(Bias-Last_Bias)/100000;//位置式PID控制器
	else if(Car_Mode == Akm_Car)
		OutPut=Akm_Along_Distance_KP*Bias/100000+Akm_Along_Distance_KI*Integral_bias/100000+Akm_Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	else if(Car_Mode == Diff_Car || Car_Mode == Tank_Car)
		OutPut=Diff_Along_Distance_KP*Bias/100+Diff_Along_Distance_KI*Integral_bias/100+Diff_Along_Distance_KD*(Bias-Last_Bias);//位置式PID控制器
	else
	  OutPut=-Along_Distance_KP*Bias/100000-Along_Distance_KI*Integral_bias/100000-Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：跟随雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//跟随雷达距离调整pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=Distance_KP*Bias/100+Distance_KI*Integral_bias/100+Distance_KD*(Bias-Last_Bias)/100;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：跟随雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/	 	
//跟随雷达转向pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //计算偏差
	Integral_bias+=Bias;	                                 				 //求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == Akm_Car || Car_Mode == Omni_Car)
		OutPut=(Follow_KP_Akm/100)*Bias+(Follow_KI_Akm/100)*Integral_bias+(Follow_KD_Akm/100)*(Bias-Last_Bias);	//位置式PID控制器
	else
	  OutPut=(Follow_KP/100)*Bias+(Follow_KI/100)*Integral_bias+(Follow_KD/100)*(Bias-Last_Bias);	//位置式PID控制器
	Last_Bias=Bias;                                       					 		//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                           					 	//输出
	
}


/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

//int Incremental_PI_Left (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//计算偏差
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//增量式PI控制器
//	 Last_bias=Bias;	                   					//保存上一次偏差 
//	 return Pwm;                         					//增量输出
//}


//int Incremental_PI_Right (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//计算偏差
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//增量式PI控制器
//	 Last_bias=Bias;	                   					//保存上一次偏差 
//	 return Pwm;                         					//增量输出
//}

