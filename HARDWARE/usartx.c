#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2 
}

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{
		u8 Usart_Receive;		
		Usart_Receive = USART_ReceiveData(USART1); //Read the data //读取数据	
	}
	return 0;	
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{	
	static int temp_count = 0;				//用于记录前进的指令的次数，第一次连接蓝牙的时候需要用到
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{	      
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //读取数据

	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //进入APP转向控制界面
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//进入APP方向控制界面	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP摇杆控制界面命令
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP转向控制界面命令
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) 
			{
				if((++temp_count) == 5)					//需要连续发送5次前进的指令，上拉转盘一段时间可开始app控制
				{
					temp_count = 0;
					APP_ON_Flag = RC_ON;		
					PS2_ON_Flag = RC_OFF;
				}
				Flag_Direction=Usart_Receive-0x40;
			}
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s
	  
	 // The following is the communication with the APP debugging interface
	 //以下是与APP调试界面通讯
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP参数指令起始位
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位

	 if(Flag_PID==1) //Collect data //采集数据
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //分析数据
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				
				if(Mode == APP_Control_Mode)
					{
					switch(Receive[1])			//APP电机调参
					 {
						 case 0x30:  RC_Velocity=Data;break;
						 case 0x31:  Velocity_KP=Data;break;
						 case 0x32:  Velocity_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == ELE_Line_Patrol_Mode)		//电磁巡线z轴速度调参
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_ELE=Data;break;
						 case 0x31:  ELE_KP=Data;break;
						 case 0x32:  ELE_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else 	if(Mode == CCD_Line_Patrol_Mode)									//CCD巡线z轴速度调参
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_CCD=Data;break;
						 case 0x31:  CCD_KP=Data;break;
						 case 0x32:  CCD_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == Lidar_Along_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30:  Along_Distance_KP=Data;break;
						 case 0x31:  Along_Distance_KD=Data;break;
						 case 0x32:  Along_Distance_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
					else if(Mode == Lidar_Follow_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30:  break;
						 case 0x31:  break;
						 case 0x32:  break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
				}		
      //Relevant flag position is cleared			
      //相关标志位清零			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//数组清零
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
	 if(RC_Velocity_CCD<0)   RC_Velocity_CCD=0;
	 if(RC_Velocity_ELE<0)   RC_Velocity_ELE=0;		 
  }
  return 0;	
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//阿克曼小车需要设置最小转弯半径
	//如果目标速度要求的转弯半径小于最小转弯半径，
	//会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//如果目标速度要求的转弯半径小于最小转弯半径
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}


