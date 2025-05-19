/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-07-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-07-05

All rights reserved
***********************************************/

#include "lidar.h"
#include <string.h>
#include "sys.h"

float Diff_Along_Distance_KP = -0.030f,Diff_Along_Distance_KD = -0.245f,Diff_Along_Distance_KI = -0.001f;	//����С����ֱ�߾������PID����
float Akm_Along_Distance_KP = -0.115f*1000,Akm_Along_Distance_KD = -1000.245f*1000,Akm_Along_Distance_KI = -0.001f*1000;	//��������ֱ�߾������PID����
float FourWheel_Along_Distance_KP = -0.115f*1000,FourWheel_Along_Distance_KD = -100.200f*1000,FourWheel_Along_Distance_KI = -0.001f*1000;	//������ֱ�߾������PID����
float Along_Distance_KP = 0.163f*1000,Along_Distance_KD = 0.123*1000,Along_Distance_KI = 0.001f*1000;	//���ֺ�ȫ����ֱ�߾������PID����

float Distance_KP = -0.150f,Distance_KD = -0.052f,Distance_KI = -0.001;	//����������PID����

float Follow_KP = -2.566f,Follow_KD = -1.368f,Follow_KI = -0.001f;  //ת��PID
float Follow_KP_Akm = -0.550f,Follow_KD_Akm = -0.121f,Follow_KI_Akm = -0.001f;
//float Distance_KP = 0.653f*1000,Distance_KD = 2.431f*1000,Distance_KI = 0.001*1000;	//�������PID����

PointDataProcessDef PointDataProcess[1200];//����225������
LiDARFrameTypeDef Pack_Data;
SpeedFrameTypeDef Pack_SpeedData; // һ����������С�����ݵİ�
PointDataProcessDef Dataprocess[1200];      //����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����
SpeedDataProcessDef SpeedDataProcess; //֧�����10��С�����ٶ�����
u8 CarCount = 0; //��ǰ���ݰ��е�С������


/**************************************************************************
Function: LIDAR_USART_Init
Input   : none
Output  : none
�������ܣ��״ﴮ�ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//����5

void LIDAR_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	USART_DeInit(UART5);  //��λ����5

	// �򿪴���GPIO��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOC Dʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	// ��USART Rx��GPIO����Ϊ��������ģʽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��

	//����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж� 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);


	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = 115200;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(UART5, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	


	// ʹ�ܴ���
	USART_Cmd(UART5, ENABLE);	    

	memset(&Pack_SpeedData, 0, sizeof(Pack_SpeedData));  // �������ṹ���ʼ��Ϊ0
	memset(&SpeedDataProcess, 0, sizeof(SpeedDataProcess));
}

///**************************************************************************
//Function: data_process
//Input   : none
//Output  : none
//�������ܣ����ݴ�����
//��ڲ�������
//����  ֵ����
//**************************************************************************/
////���һ֡���պ���д���
//int data_cnt = 0;
//void data_process(void) //���ݴ���
//{
//	int m;
//	u32 distance_sum[32]={0};//2����ľ���͵�����
//	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//����32����Ŀ�ʼ�Ƕ�
//	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//����32����Ľ����Ƕ�
//	float area_angle[32]={0};
//	
//	if(start_angle>end_angle)//�����ǶȺͿ�ʼ�Ƕȱ�0�ȷָ�����
//		end_angle +=360;

//	for(m=0;m<32;m++)
//	{
//		area_angle[m]=start_angle+(end_angle-start_angle)/32*m;
//		if(area_angle[m]>360)  area_angle[m] -=360;
//		
//		distance_sum[m] +=((u16)Pack_Data.point[m].distance_h<<8)+Pack_Data.point[m].distance_l;//���ݸߵ�8λ�ϲ�

//		Dataprocess[data_cnt].angle=area_angle[m];
//		Dataprocess[data_cnt++].distance=distance_sum[m];  //һ֡����Ϊ32����
//		if(data_cnt == 1152) data_cnt = 0;
//	}
//	
//		
//}

/**************************************************************************
Function: data_process
Input   : none
Output  : none
�������ܣ����ݴ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//���һ֡���պ���д���
int data_cnt = 0;
void data_process(uint8_t reset) //���ݴ���
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
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void UART5_IRQHandler(void)
{	
	static u8 state = 0;//״̬λ	
	u8 temp_data;
	u8 reset = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //���յ�����
	{	
		temp_data=USART_ReceiveData(UART5);	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//ͷ�̶�
				{
					Pack_SpeedData.header_0 = temp_data;
					state++;
				} else state = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//ͷ�̶�
				{
					Pack_SpeedData.header_1 = temp_data;
					state++;
				} else state = 0;
				break;
			case 2:
				Pack_SpeedData.car_count = temp_data & 0x0F; //��ȡ���ٶȵ�С������ (0-5)
				if(Pack_SpeedData.car_count > 5) Pack_SpeedData.car_count = 5; //�����������Ϊ5
				if(Pack_SpeedData.car_count == 0) {
					state = 38; // ���û��С�����ݣ�ֱ����ת����β����
				} else {
					state = 3; // ��ʼ�����һ������
				}
				break;
			case 3: case 10: case 17: case 24: case 31:
				// ��ȡС��ID
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
				// ����Ƿ���Ҫ������һ������
				if(state == 10 && Pack_SpeedData.car_count > 1) {
					state = 10; // ����ڶ�������
				}
				else if(state == 17 && Pack_SpeedData.car_count > 2) {
					state = 17; // �������������
				}
				else if(state == 24 && Pack_SpeedData.car_count > 3) {
					state = 24; // ������Ķ�����
				}
				else if(state == 31 && Pack_SpeedData.car_count > 4) {
					state = 31; // ������������
				}
				else {
					state = 38; // �������ݴ�����ϣ���ת����β����
				}
				break;
			case 38:
				if(temp_data == TAIL_0)//β�̶�
				{
					Pack_SpeedData.tail_0 = temp_data;
					state++;
				} else state = 0;
				break;			
			case 39:
				if(temp_data == TAIL_1 && Pack_SpeedData.car_count != 0)//β�̶�
				{
					Pack_SpeedData.tail_1 = temp_data;
				} 
				else
				{
					memset(&Pack_SpeedData, 0, sizeof(Pack_SpeedData));//����
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
�������ܣ���ֱ���״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//��ֱ���״�������pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == FourWheel_Car)
		OutPut=FourWheel_Along_Distance_KP*Bias/100000+FourWheel_Along_Distance_KI*Integral_bias/100000+FourWheel_Along_Distance_KD*(Bias-Last_Bias)/100000;//λ��ʽPID������
	else if(Car_Mode == Akm_Car)
		OutPut=Akm_Along_Distance_KP*Bias/100000+Akm_Along_Distance_KI*Integral_bias/100000+Akm_Along_Distance_KD*(Bias-Last_Bias)/1000;//λ��ʽPID������
	else if(Car_Mode == Diff_Car || Car_Mode == Tank_Car)
		OutPut=Diff_Along_Distance_KP*Bias/100+Diff_Along_Distance_KI*Integral_bias/100+Diff_Along_Distance_KD*(Bias-Last_Bias);//λ��ʽPID������
	else
	  OutPut=-Along_Distance_KP*Bias/100000-Along_Distance_KI*Integral_bias/100000-Along_Distance_KD*(Bias-Last_Bias)/1000;//λ��ʽPID������
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	
}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ������״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//�����״�������pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=Distance_KP*Bias/100+Distance_KI*Integral_bias/100+Distance_KD*(Bias-Last_Bias)/100;//λ��ʽPID������
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
�������ܣ������״�ת��pid
��ڲ���: ��ǰ�ǶȺ�Ŀ��Ƕ�
����  ֵ�����ת���ٶ�
**************************************************************************/	 	
//�����״�ת��pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //����ƫ��
	Integral_bias+=Bias;	                                 				 //���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == Akm_Car || Car_Mode == Omni_Car)
		OutPut=(Follow_KP_Akm/100)*Bias+(Follow_KI_Akm/100)*Integral_bias+(Follow_KD_Akm/100)*(Bias-Last_Bias);	//λ��ʽPID������
	else
	  OutPut=(Follow_KP/100)*Bias+(Follow_KI/100)*Integral_bias+(Follow_KD/100)*(Bias-Last_Bias);	//λ��ʽPID������
	Last_Bias=Bias;                                       					 		//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���
	
}


/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/

//int Incremental_PI_Left (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//����ƫ��
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//����ʽPI������
//	 Last_bias=Bias;	                   					//������һ��ƫ�� 
//	 return Pwm;                         					//�������
//}


//int Incremental_PI_Right (int Encoder,int Target)
//{ 	
//	 static int Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder;                					//����ƫ��
//	 Pwm+=11*(Bias-Last_bias)+10*Bias;   	//����ʽPI������
//	 Last_bias=Bias;	                   					//������һ��ƫ�� 
//	 return Pwm;                         					//�������
//}

