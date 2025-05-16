#include "balance.h"

int Time_count=0; //Time variable //¼ÆÊ±±äÁ¿ 

u8 Lidar_Detect = Lidar_Detect_ON;			//µç´ÅÑ²ÏßÄ£Ê½À×´ï¼ì²âÕÏ°­Îï£¬Ä¬ÈÏ¿ªÆô

u8 Mode;
float RC_Velocity_CCD=350,RC_Velocity_ELE=350; 
float PS2_Velocity,PS2_Turn_Velocity;			//Ò£¿Ø¿ØÖÆµÄËÙ¶È
Encoder OriginalEncoder; //Encoder raw data //±àÂëÆ÷Ô­Ê¼Êı¾İ     
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
º¯Êı¹¦ÄÜ£ºÔË¶¯Ñ§Äæ½â£¬¸ù¾İÈıÖáÄ¿±êËÙ¶È¼ÆËã¸÷³µÂÖÄ¿±ê×ªËÙ
Èë¿Ú²ÎÊı£ºXºÍY¡¢ZÖá·½ÏòµÄÄ¿±êÔË¶¯ËÙ¶È
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //³µÂÖÄ¿±êËÙ¶ÈÏŞ·ù
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //È«ÏòÒÆ¶¯Ğ¡³µ²Å¿ªÆôËÙ¶ÈÆ½»¬´¦Àí
	   
	
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //¶ÔÊäÈëËÙ¶È½øĞĞÆ½»¬´¦Àí
  
      //Get the smoothed data 
			//»ñÈ¡Æ½»¬´¦ÀíºóµÄÊı¾İ			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //Âó¿ËÄÉÄ·ÂÖĞ¡³µ
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//È«ÏòÂÖĞ¡³µ
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //Ã»ÓĞÊ¹ÓÃµ½
		}
		
		//Ackermann structure car
		//°¢¿ËÂüĞ¡³µ
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //°¢¿ËÂüĞ¡³µ×¨ÓÃÏà¹Ø±äÁ¿
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//¶ÔÓÚ°¢¿ËÂüĞ¡³µVz´ú±íÓÒÇ°ÂÖ×ªÏò½Ç¶È
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//Ç°ÂÖ×ªÏò½Ç¶ÈÏŞ·ù(¶æ»ú¿ØÖÆÇ°ÂÖ×ªÏò½Ç¶È)£¬µ¥Î»£ºrad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//¶æ»úPWMÖµ£¬¶æ»ú¿ØÖÆÇ°ÂÖ×ªÏò½Ç¶È
			//Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.755f;
			Servo=SERVO_INIT + (Angle_Servo - 1.755f)*Ratio;

			
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
			MOTOR_D.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //¶æ»úPWMÖµÏŞ·ù
			}
		
		//Differential car
		//²îËÙĞ¡³µ
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //¼ÆËã³ö×óÂÖµÄÄ¿±êËÙ¶È
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //¼ÆËã³öÓÒÂÖµÄÄ¿±êËÙ¶È
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
			MOTOR_D.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
			
		}
		
		//FourWheel car
		//ËÄÇı³µ
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //¼ÆËã³ö×óÂÖµÄÄ¿±êËÙ¶È
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //¼ÆËã³ö×óÂÖµÄÄ¿±êËÙ¶È
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //¼ÆËã³öÓÒÂÖµÄÄ¿±êËÙ¶È
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //¼ÆËã³öÓÒÂÖµÄÄ¿±êËÙ¶È
					
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 		
		}
		
		//Tank Car
		//ÂÄ´ø³µ
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //ÔË¶¯Ñ§Äæ½â
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //¼ÆËã³ö×óÂÖµÄÄ¿±êËÙ¶È
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //¼ÆËã³öÓÒÂÖµÄÄ¿±êËÙ¶È
			
			//Wheel (motor) target speed limit //³µÂÖ(µç»ú)Ä¿±êËÙ¶ÈÏŞ·ù
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
			MOTOR_D.Target=0; //Out of use //Ã»ÓĞÊ¹ÓÃµ½
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£ºFreeRTOSÈÎÎñ£¬ºËĞÄÔË¶¯¿ØÖÆÈÎÎñ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  static u8 Count_CCD = 0;								//µ÷½ÚCCD¿ØÖÆÆµÂÊ
	  static u8 last_mode = 0;
	  u32 lastWakeTime = getSysTickCnt();
	  
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//´ËÈÎÎñÒÔ100HzµÄÆµÂÊÔËĞĞ£¨10ms¿ØÖÆÒ»´Î£©
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			//Time count is no longer needed after 30 seconds
			//Ê±¼ä¼ÆÊı£¬30Ãëºó²»ÔÙĞèÒª
			if(Time_count<3000)Time_count++;
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//»ñÈ¡±àÂëÆ÷Êı¾İ£¬¼´³µÂÖÊµÊ±ËÙ¶È£¬²¢×ª»»Î»¹ú¼Êµ¥Î»
			Get_Velocity_Form_Encoder();   

        switch(click_N_Double(50))
				{
					case 1:    //µ¥»÷ÓÃÀ´ÇĞ»»Ä£Ê½
						Mode+=1;
						if(Mode == ELE_Line_Patrol_Mode)			//Ñ¡Ôñµç´ÅÑ²Ïß¿ØÖÆÄ£Ê½
						{
								ele_Init();							//³õÊ¼»¯ELE
						}
						else if(Mode == CCD_Line_Patrol_Mode)			//Ñ¡Ôñµç´ÅÑ²Ïß¿ØÖÆÄ£Ê½
						{
								ccd_Init();							//³õÊ¼»¯ELE
						}
					else if(Mode>6)
							 Mode = 0;
						 break;
					case 2:    //µç´ÅÑ²Ïß×´Ì¬Ê±£¬Ë«»÷¿ÉÒÔ´ò¿ª/¹Ø±ÕÀ×´ï¼ì²âÕÏ°­Îï£¬Ä¬ÈÏ´ò¿ª
						Lidar_Detect = !Lidar_Detect;
						if(Lidar_Detect == Lidar_Detect_OFF)
							memset(Dataprocess,0, sizeof(PointDataProcessDef)*225);		//ÓÃÓÚÀ×´ï¼ì²âÕÏ°­ÎïµÄÊı×éÇåÁã
						break;				 
			  }
				
				if(last_mode != Mode)  //????????????
				{
					last_mode++;
					OLED_Clear();
					if(last_mode>6)
					{
						//OLED_Clear();
						last_mode = 0;
					}
				} 
      			
				if(Mode != ELE_Line_Patrol_Mode)
					Buzzer_Alarm(0);
			  if(Mode == APP_Control_Mode)          Get_RC();             //Handle the APP remote commands //´¦ÀíAPPÒ£¿ØÃüÁî
			  else if(Mode == PS2_Control_Mode)     PS2_Control();        //Handle PS2 controller commands //´¦ÀíPS2ÊÖ±ú¿ØÖÆÃüÁî
//			  else if(Mode == Lidar_Avoid_Mode)     Lidar_Avoid();        //Avoid Mode //±ÜÕÏÄ£Ê½
				else if(Mode == Lidar_Avoid_Mode)     Get_Speed();        //Avoid Mode //±ÜÕÏÄ£Ê½
			  else if(Mode == Lidar_Follow_Mode)    Lidar_Follow();       //Follow Mode //¸úËæÄ£Ê½
				else if(Mode == Lidar_Along_Mode)     Lidar_along_wall();   //Along Mode //×ßÖ±ÏßÄ£Ê½
				else if(Mode == ELE_Line_Patrol_Mode) 
				{	
				  Get_RC_ELE();         //ELEÄ£Ê
				}
				else														//CCDÄ£Ê½
					{
						if(++Count_CCD == 4)									//µ÷½Ú¿ØÖÆÆµÂÊ£¬4*5 = 20ms¿ØÖÆÒ»´Î
						{
							Count_CCD = 0;
							Get_RC_CCD();											
						}
						else if(Count_CCD>4)
							Count_CCD = 0;
					}					
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//Èç¹ûµç³ØµçÑ¹²»´æÔÚÒì³££¬¶øÇÒÊ¹ÄÜ¿ª¹ØÔÚONµµÎ»£¬¶øÇÒÈí¼şÊ§ÄÜ±êÖ¾Î»Îª0
				if(Turn_Off(Voltage)==0) 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //ËÙ¶È±Õ»·¿ØÖÆ¼ÆËã¸÷µç»úPWMÖµ£¬PWM´ú±í³µÂÖÊµ¼Ê×ªËÙ
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
					 Limit_Pwm(16500) ;
					 //Set different PWM control polarity according to different car models
					 //¸ù¾İ²»Í¬Ğ¡³µĞÍºÅÉèÖÃ²»Í¬µÄPWM¿ØÖÆ¼«ĞÔ
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //Âó¿ËÄÉÄ·ÂÖĞ¡³µ
							case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //È«ÏòÂÖĞ¡³µ
							case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //°¢¿ËÂüĞ¡³µ
							case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //Á½ÂÖ²îËÙĞ¡³µ
							case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //ËÄÇı³µ 
							case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //ÂÄ´ø³µ
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //Èç¹ûTurn_Off(Voltage)·µ»ØÖµÎª1£¬²»ÔÊĞí¿ØÖÆĞ¡³µ½øĞĞÔË¶¯£¬PWMÖµÉèÖÃÎª0
				 else	Set_Pwm(0,0,0,0,0); 
			 	
		 }  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
º¯Êı¹¦ÄÜ£º¸³Öµ¸øPWM¼Ä´æÆ÷£¬¿ØÖÆ³µÂÖ×ªËÙÓë·½Ïò
Èë¿Ú²ÎÊı£ºPWM
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	if(motor_a<0)			PWMA2=16800,PWMA1=16800+motor_a;
	else 	            PWMA1=16800,PWMA2=16800-motor_a;
			
	if(motor_b<0)			PWMB1=16800,PWMB2=16800+motor_b;
	else 	            PWMB2=16800,PWMB1=16800-motor_b;
			
	if(motor_c<0)			PWMC1=16800,PWMC2=16800+motor_c;
	else 	            PWMC2=16800,PWMC1=16800-motor_c;
	
	if(motor_d<0)			PWMD2=16800,PWMD1=16800+motor_d;
	else 	            PWMD1=16800,PWMD2=16800-motor_d;
	
	//Servo control
	//¶æ»ú¿ØÖÆ
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
º¯Êı¹¦ÄÜ£ºÏŞÖÆPWMÖµ 
Èë¿Ú²ÎÊı£º·ùÖµ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
º¯Êı¹¦ÄÜ£ºÏŞ·ùº¯Êı
Èë¿Ú²ÎÊı£º·ùÖµ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
º¯Êı¹¦ÄÜ£º¼ì²éµç³ØµçÑ¹¡¢Ê¹ÄÜ¿ª¹Ø×´Ì¬¡¢Èí¼şÊ§ÄÜ±êÖ¾Î»×´Ì¬
Èë¿Ú²ÎÊı£ºµçÑ¹
·µ»Ø  Öµ£ºÊÇ·ñÔÊĞí¿ØÖÆ£¬1£º²»ÔÊĞí£¬0ÔÊĞí
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1; 
					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
º¯Êı¹¦ÄÜ£ºÇó¾ø¶ÔÖµ
Èë¿Ú²ÎÊı£ºlong int
·µ»Ø  Öµ£ºunsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)

º¯Êı¹¦ÄÜ£ºÔöÁ¿Ê½PI¿ØÖÆÆ÷
Èë¿Ú²ÎÊı£º±àÂëÆ÷²âÁ¿Öµ(Êµ¼ÊËÙ¶È)£¬Ä¿±êËÙ¶È
·µ»Ø  Öµ£ºµç»úPWM
¸ù¾İÔöÁ¿Ê½ÀëÉ¢PID¹«Ê½ 
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)´ú±í±¾´ÎÆ«²î 
e(k-1)´ú±íÉÏÒ»´ÎµÄÆ«²î  ÒÔ´ËÀàÍÆ 
pwm´ú±íÔöÁ¿Êä³ö
ÔÚÎÒÃÇµÄËÙ¶È¿ØÖÆ±Õ»·ÏµÍ³ÀïÃæ£¬Ö»Ê¹ÓÃPI¿ØÖÆ
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶ÔAPPÍ¨¹ı´®¿Ú2·¢ËÍ¹ıÀ´µÄÃüÁî½øĞĞ´¦Àí
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //È«ÏòÂÖÔË¶¯Ğ¡³µ¿ÉÒÔ½øĞĞºáÏòÒÆ¶¯
	{
	 switch(Flag_Direction)  //Handle direction control commands //´¦Àí·½Ïò¿ØÖÆÃüÁî
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //Èç¹ûÎŞ·½Ïò¿ØÖÆÖ¸Áî£¬¼ì²é×ªÏò¿ØÖÆ×´Ì¬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //×ó×Ô×ª  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //ÓÒ×Ô×ª
		 else 		               Move_Z=0;                       //stop           //Í£Ö¹
	 }
	}	
	else //Non-omnidirectional moving trolley //·ÇÈ«ÏòÒÆ¶¯Ğ¡³µ
	{
	 switch(Flag_Direction) //Handle direction control commands //´¦Àí·½Ïò¿ØÖÆÃüÁî
	 { 
			case 1:     Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //×ó×Ô×ª 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //ÓÒ×Ô×ª	
	}
	
	//Z-axis data conversion //ZÖáÊı¾İ×ª»¯
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//°¢¿ËÂü½á¹¹Ğ¡³µ×ª»»ÎªÇ°ÂÖ×ªÏò½Ç¶È
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //²îËÙ¿ØÖÆÔ­ÀíÏµÁĞĞèÒª´Ë´¦Àí
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //µ¥Î»×ª»»£¬mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

void Get_Speed(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //È«ÏòÂÖÔË¶¯Ğ¡³µ¿ÉÒÔ½øĞĞºáÏòÒÆ¶¯
	{
	 
	 Move_X = SpeedDataProcess.speed_x * 100.0f;
	 if (SpeedDataProcess.dir_x == 0x01 ) Move_X = -1 * Move_X;
	 Move_Y = SpeedDataProcess.speed_y * 100.0f;
	 if (SpeedDataProcess.dir_y == 0x01 ) Move_Y = -1 * Move_Y;
	 Move_Z = SpeedDataProcess.speed_z * 1.0f;
	 if (SpeedDataProcess.dir_z == 0x01 ) Move_Z = -1 * Move_Z;
	 
//	 if(Flag_Move==0)		
//	 {	
//		 //If no direction control instruction is available, check the steering control status
//		 //Èç¹ûÎŞ·½Ïò¿ØÖÆÖ¸Áî£¬¼ì²é×ªÏò¿ØÖÆ×´Ì¬
//		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //×ó×Ô×ª  
//		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //ÓÒ×Ô×ª
//		 else 		               Move_Z=0;                       //stop           //Í£Ö¹
//	 }
	}
	
		//Unit conversion, mm/s -> m/s
  //µ¥Î»×ª»»£¬mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
	Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º¶ÁÈ¡±àÂëÆ÷ÊıÖµ²¢¼ÆËã³µÂÖËÙ¶È£¬µ¥Î»m/s
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //»ñÈ¡±àÂëÆ÷µÄÔ­Ê¼Êı¾İ
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//¸ù¾İ²»Í¬Ğ¡³µĞÍºÅ¾ö¶¨±àÂëÆ÷ÊıÖµ¼«ĞÔ
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//±àÂëÆ÷Ô­Ê¼Êı¾İ×ª»»Îª³µÂÖËÙ¶È£¬µ¥Î»m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
º¯Êı¹¦ÄÜ£º¶ÔÈıÖáÄ¿±êËÙ¶È×öÆ½»¬´¦Àí
Èë¿Ú²ÎÊı£ºÈıÖáÄ¿±êËÙ¶È
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
º¯Êı¹¦ÄÜ£º¸¡µãĞÍÊı¾İ¼ÆËã¾ø¶ÔÖµ
Èë¿Ú²ÎÊı£º¸¡µãÊı
·µ»Ø  Öµ£ºÊäÈëÊıµÄ¾ø¶ÔÖµ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£ºPS2ÊÖ±ú¿ØÖÆ
Èë¿Ú²ÎÊı: ÎŞ 
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/	 	
void PS2_Control(void)
{
	int LY,RX,LX;									//ÊÖ±úADCµÄÖµ
	int Threshold=20; 							//ãĞÖµ£¬ºöÂÔÒ¡¸ËĞ¡·ù¶È¶¯×÷
	static float Key1_Count = 0,Key2_Count = 0;	//ÓÃÓÚ¿ØÖÆ¶ÁÈ¡Ò¡¸ËµÄËÙ¶È
	//×ª»¯Îª128µ½-128µÄÊıÖµ
	LY=-(PS2_LY-128);//×ó±ßYÖá¿ØÖÆÇ°½øºóÍË
	RX=-(PS2_RX-128);//ÓÒ±ßXÖá¿ØÖÆ×ªÏò
	LX=-(PS2_LX-128);//×ó±ßXÖá¿ØÖÆ×ªÏò,ÂóÂÖºÍÈ«ÏòĞ¡³µ×¨ÓÃ
	
	if(LY>-Threshold&&LY<Threshold)	LY=0;
	if(RX>-Threshold&&RX<Threshold)	RX=0;		//ºöÂÔÒ¡¸ËĞ¡·ù¶È¶¯×÷
	if(LX>-Threshold&&LX<Threshold)	LX=0;
	
	if(Strat) //°´ÏÂstart¼ü²Å¿ÉÒÔ¿ØÖÆĞ¡³µ
	{
		if (PS2_KEY == PSB_L1) 					 	//°´ÏÂ×ó1¼ü¼ÓËÙ£¨°´¼üÔÚ¶¥ÉÏ£©
		{	
			if((++Key1_Count) == 20)				//µ÷½Ú°´¼ü·´Ó¦ËÙ¶È
			{
				PS2_KEY = 0;
			  Key1_Count = 0;
				if((PS2_Velocity += X_Step)>MAX_RC_Velocity)				//Ç°½ø×î´óËÙ¶È1230
					PS2_Velocity = MAX_RC_Velocity;
				if(Car_Mode != Akm_Car)								//·Ç°¢¿ËÂü³µ¿Éµ÷½Ú×ªÏòËÙ¶È
				{
					if((PS2_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//×ªÏò×î´óËÙ¶È325
						PS2_Turn_Velocity = MAX_RC_Turn_Bias;
				}
			}
		}
		else if(PS2_KEY == PSB_R1) 					//°´ÏÂÓÒ1¼ü¼õËÙ
		{
			if((++Key2_Count) == 15)
			{
				PS2_KEY = 0;
				Key2_Count = 0;
				if((PS2_Velocity -= X_Step)<MINI_RC_Velocity)			//Ç°ºó×îĞ¡ËÙ¶È110
					PS2_Velocity = MINI_RC_Velocity;
				
				if(Car_Mode != Akm_Car)								//·Ç°¢¿ËÂü³µ¿Éµ÷½Ú×ªÏòËÙ¶È
				{
					if((PS2_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//×ªÏò×îĞ¡ËÙ¶È45
					PS2_Turn_Velocity = MINI_RC_Turn_Velocity;
				}
			}
		}
		else
			Key2_Count = 0,Key2_Count = 0;			//¶ÁÈ¡µ½ÆäËû°´¼üÖØĞÂ¼ÆÊı
		Move_X = (PS2_Velocity/128)*LY;				//ËÙ¶È¿ØÖÆ£¬Á¦¶È±íÊ¾ËÙ¶È´óĞ¡
		if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)
		{
			Move_Y = LX*PS2_Velocity/128;
		}
		else
		{
			Move_Y = 0;
		}
		if(Car_Mode == Akm_Car)						//°¢¿ËÂü³µ×ªÏò¿ØÖÆ£¬Á¦¶È±íÊ¾×ªÏò½Ç¶È
			Move_Z = (PS2_Turn_Velocity/128)*RX;	
		else										//ÆäËû³µĞÍ×ªÏò¿ØÖÆ
		{
			//if(Move_X>=0)
				Move_Z = (PS2_Turn_Velocity/128)*RX;	//×ªÏò¿ØÖÆ£¬Á¦¶È±íÊ¾×ªÏòËÙ¶È
//			else
//				Move_Z = -(PS2_Turn_Velocity/128)*RX;
		}
  }
	else
	{
		Move_X = 0;
		Move_Y = 0;
		Move_Z = 0;
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


/**************************************************************************
º¯Êı¹¦ÄÜ£ºCCDÑ²Ïß£¬²É¼¯3¸öµç¸ĞµÄÊı¾İ²¢ÌáÈ¡ÖĞÏß 
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void  Get_RC_CCD(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
									
			Move_X=RC_Velocity_CCD;													//CCDÑ²ÏßÄ£Ê½ÏßËÙ¶È
			Bias=CCD_Median-64;  //ÌáÈ¡Æ«²î£¬64ÎªÑ²ÏßµÄÖĞĞÄµã
	    if(Car_Mode == Omni_Car)
			  move_z=-Bias*Omni_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Omni_Car_CCD_KI*0.1f; //PD¿ØÖÆ£¬Ô­Àí¾ÍÊÇÊ¹µÃĞ¡³µ±£³Ö¿¿½üÑ²ÏßµÄÖĞĞÄµã
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Tank_Car_CCD_KI*0.1f;
			else
				move_z=-Bias*CCD_KP*0.1f-(Bias-Last_Bias)*CCD_KI*0.1f;
			Last_Bias=Bias;   //±£´æÉÏÒ»´ÎµÄÆ«²î
			if(Car_Mode==Mec_Car)															
			{
				Move_Z=move_z*RC_Velocity_CCD/50000;							//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}

			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_CCD/21000;							//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Akm_Car)												
			{
				Move_Z=move_z/450;																//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Diff_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/67000;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí	
			}
			else if(Car_Mode==Tank_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/50000;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí	
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/20100;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}			
		
			//Z-axis data conversion //ZÖáÊı¾İ×ª»¯	
			//Unit conversion, mm/s -> m/s
			//µ¥Î»×ª»»£¬mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
			//Control target value is obtained and kinematics analysis is performed
			//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
º¯Êı¹¦ÄÜ£ºµç´ÅÑ²Ïß£¬²É¼¯3¸öµç¸ĞµÄÊı¾İ²¢ÌáÈ¡ÖĞÏß 
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void  Get_RC_ELE(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
	
	if(Detect_Barrier() == No_Barrier)
	{
			Move_X=RC_Velocity_ELE;				//µç´ÅÑ²ÏßÄ£Ê½µÄËÙ¶È
			Bias=100-Sensor;  //ÌáÈ¡Æ«²î	
      if(Car_Mode == Omni_Car)		
			  move_z=-Bias* Omni_Car_ELE_KP*0.08f-(Bias-Last_Bias)* Omni_Car_ELE_KI*0.05f; 
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_ELE_KP*0.1f-(Bias-Last_Bias)*Tank_Car_ELE_KI*0.1f;
			else
				move_z=-Bias* ELE_KP*0.08f-(Bias-Last_Bias)* ELE_KI*0.05f; 
			Last_Bias=Bias; 
		  Buzzer_Alarm(0);

			if(Car_Mode==Mec_Car)															
			{		
				Move_Z=move_z*RC_Velocity_ELE/50000;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_ELE/10800;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Diff_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/45000;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Tank_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/28000;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;
				Move_Z=move_z*RC_Velocity_ELE/20100;					//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
			
			else if(Car_Mode==Akm_Car)											
			{
				Move_Z=move_z/450;														//²îËÙ¿ØÖÆÔ­ÀíĞèÒª¾­¹ı´Ë´¦´¦Àí
			}
		}
	
	else									//ÓĞÕÏ°­Îï
	{
		Buzzer_Alarm(100);				//µ±µç»úÊ¹ÄÜµÄÊ±ºò£¬ÓĞÕÏ°­ÎïÔò·äÃùÆ÷±¨¾¯
		Move_X = 0;
		Move_Z = 0;
	}

			//Z-axis data conversion //ZÖáÊı¾İ×ª»¯	
			//Unit conversion, mm/s -> m/s
			//µ¥Î»×ª»»£¬mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
	
			//Control target value is obtained and kinematics analysis is performed
			//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
	    Move_Y=0;
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
º¯Êı¹¦ÄÜ£º¼ì²âÇ°·½ÊÇ·ñÓĞÕÏ°­Îï
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
u8 Detect_Barrier(void)
{
	int i;
	u8 point_count = 0;
	
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<1152;i++)	//¼ì²âÊÇ·ñÓĞÕÏ°­Îï 
		{
			if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<700)//700mmÄÚÊÇ·ñÓĞÕÏ°­Îï
					point_count++;
		  }
	}
		if(point_count > 0)//ÓĞÕÏ°­Îï
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}

/**************************************************************************
º¯Êı¹¦ÄÜ£ºĞ¡³µ±ÜÕÏÄ£Ê½
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Lidar_Avoid(void)
{
	int i = 0; 
	u8 calculation_angle_cnt = 0;	//ÓÃÓÚÅĞ¶Ï225¸öµãÖĞĞèÒª×ö±ÜÕÏµÄµã
	float angle_sum = 0;			//´ÖÂÔ¼ÆËãÕÏ°­ÎïÎ»ÓÚ×ó»òÕßÓÒ
	u8 distance_count = 0;			//¾àÀëĞ¡ÓÚÄ³ÖµµÄ¼ÆÊı
	for(i=0;i<1152;i++)				//±éÀú120¶È·¶Î§ÄÚµÄ¾àÀëÊı¾İ£¬¹²120¸öµã×óÓÒµÄÊı¾İ
	{
		if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))  //±ÜÕÏ½Ç¶ÈÔÚ300-60Ö®¼ä
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Avoid_Distance))	//¾àÀëĞ¡ÓÚ450mmĞèÒª±ÜÕÏ,Ö»ĞèÒª120¶È·¶Î§ÄÚµã
			{
				calculation_angle_cnt++;						 			//¼ÆËã¾àÀëĞ¡ÓÚ±ÜÕÏ¾àÀëµÄµã¸öÊı
				if(Dataprocess[i].angle<60)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>300)
					angle_sum += (Dataprocess[i].angle-360);	//300¶Èµ½60¶È×ª»¯Îª-60¶Èµ½60¶È
				if(Dataprocess[i].distance<Avoid_Min_Distance)				//¼ÇÂ¼Ğ¡ÓÚ200mmµÄµãµÄ¼ÆÊı
					distance_count++;
			}
	  }
	}
	Move_X = forward_velocity;
  if(calculation_angle_cnt == 0)//²»ĞèÒª±ÜÕÏ
	 {
		Move_Z = 0;
	 }
	else                          //µ±¾àÀëĞ¡ÓÚ200mm£¬Ğ¡³µÍùºóÍË
	{
		if(distance_count>8)
		{
			Move_X = -forward_velocity;
			Move_Z = 0;
		}
		else
		{
			Move_X = 0;
			if(angle_sum > 0)//ÕÏ°­ÎïÆ«ÓÒ
			{
				if(Car_Mode == Mec_Car)  //ÂóÂÖ×ªÍäĞèÒª°ÑÇ°½øËÙ¶È½µµÍ
					Move_X = 0;
				else                     //ÆäËû³µĞÍ±£³ÖÔ­ÓĞ³µËÙ
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=corner_velocity;
				else
				  Move_Z=other_corner_velocity;//×ó×ª
			}
			else //Æ«×ó
			{
				if(Car_Mode == Mec_Car)
					Move_X = 0;
				else
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = -PI/4;
				else if(Car_Mode == Omni_Car)
				  Move_Z=-corner_velocity;//ÓÒ×ª
				else
					Move_Z=-other_corner_velocity;
			}
	  }
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}



/**************************************************************************
º¯Êı¹¦ÄÜ£ºĞ¡³µ¸úËæÄ£Ê½ 
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//±ÜÕÏµÄ½Ç¶È
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//ÓÃÓÚÂË³ıÒ»Ğ´ÔëµãµÄ¼ÆÊı±äÁ¿
	//ĞèÒªÕÒ³ö¸úËæµÄÄÇ¸öµãµÄ½Ç¶È
	for(i = 0; i < 1152; i++)
	{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Follow_Distance))
			{
				calculation_angle_cnt++;
				if(Dataprocess[i].distance<mini_distance)
				{
					mini_distance = Dataprocess[i].distance;
					angle = Dataprocess[i].angle;
				}
			}
	}
	if(angle > 180)  //0--360¶È×ª»»³É0--180£»-180--0£¨Ë³Ê±Õë£©
		angle -= 360;
	if((angle-last_angle > 10)||(angle-last_angle < -10))   //×öÒ»¶¨Ïû¶¶£¬²¨¶¯´óÓÚ10¶ÈµÄĞèÒª×öÅĞ¶Ï
	{
		if(++data_count > 30)   //Á¬Ğø30´Î²É¼¯µ½µÄÖµ(300msºó)ºÍÉÏ´ÎµÄ±È´óÓÚ10¶È£¬´ËÊ±²ÅÊÇÈÏÎªÊÇÓĞĞ§Öµ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else    //²¨¶¯Ğ¡ÓÚ10¶ÈµÄ¿ÉÒÔÖ±½ÓÈÏÎªÊÇÓĞĞ§Öµ
	{
		if(++data_count > 10)   //Á¬Ğø10´Î²É¼¯µ½µÄÖµ(100msºó)£¬´ËÊ±²ÅÊÇÈÏÎªÊÇÓĞĞ§Öµ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	if(calculation_angle_cnt < 8)  //¸úËæ¾àÀëĞ¡ÓÚ8ÇÒµ±cnt>40µÄÊ±ºò£¬ÈÏÎªÔÚ1600ÄÚÃ»ÓĞ¸úËæÄ¿±ê
	{
		if(cnt < 40)
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Move_X > 0.06f || Move_X < -0.06f)  //µ±Move_XÓĞËÙ¶ÈÊ±£¬×ªÏòPID¿ªÊ¼µ÷Õû
		{
			if(mini_distance < 700 && (last_angle > 60 || last_angle < -60))
			{
				Move_Z = -0.0098f*last_angle;  //µ±¾àÀëÆ«Ğ¡ÇÒ½Ç¶È²î¾à¹ı´óÖ±½Ó¿ìËÙ×ªÏò
			}
			else
			{
				  Move_Z = -Follow_Turn_PID(last_angle,0);		//×ªÏòPID£¬³µÍ·ÓÀÔ¶¶Ô×Å¸úËæÎïÆ·
			}
		}
		else
		{
			Move_Z = 0;
		}
		if(angle>150 || angle<-150)  //Èç¹ûĞ¡³µÔÚºó·½60¡ãĞèÒª·´·½ÏòÔË¶¯ÒÔ¼°¿ìËÙ×ªÍä
		{
			Move_X = -Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);
			Move_Z = -0.0098f*last_angle;
		}
		else
		{
		  Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //±£³Ö¾àÀë±£³ÖÔÚ500mm
		}
		Move_X = target_limit_float(Move_X,-amplitude_limiting,amplitude_limiting);   //¶ÔÇ°½øËÙ¶ÈÏŞ·ù
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
º¯Êı¹¦ÄÜ£ºĞ¡³µ×ßÖ±ÏßÄ£Ê½
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Lidar_along_wall(void)
{
	static u32 target_distance=0;
	static int i;

	u32 distance;
	u8 data_count = 0;			//ÓÃÓÚÂË³ıÒ»Ğ´ÔëµãµÄ¼ÆÊı±äÁ¿
	
	int j;
	
	Move_X = forward_velocity;  //³õÊ¼ËÙ¶È
	
	for(j=0;j<1152;j++) //225
	  {
			if(Dataprocess[j].angle>268 && Dataprocess[j].angle<272)   //È¡À×´ïµÄ4¶ÈµÄµã
			{
				if(i==0)
				{
					target_distance=Dataprocess[j].distance;  //À×´ï²¶»ñµÚÒ»¸ö¾àÀë
					i++;
				}
				 if(Dataprocess[j].distance<(target_distance+limit_distance))//ÏŞÖÆÒ»ÏÂÀ×´ïµÄÌ½²â¾àÀë
				 {
					 data_count++;
					 distance=Dataprocess[j].distance;//ÊµÊ±¾àÀë
				 }
		  }
	  }
		if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)  //Ö»ÓĞÂóÂÖºÍÈ«Ïò¿ÉÒÔÓÃMove_Y
		{
			Move_Y=Along_Adjust_PID(distance,target_distance);
			Move_X = forward_velocity;
			Move_Z = 0;
		}
		else   //ÆäËû³µĞÍÊ¹ÓÃMove_Z±£³Ö×ßÖ±Ïß×´Ì¬
		{
			Move_Z=Along_Adjust_PID(distance,target_distance);
			Move_X = forward_velocity;
			Move_Y = 0;
		}
		if(data_count == 0)  //µ±data_countµÈÓÚ0£¬Ö»ÓĞÇ°½øËÙ¶È
			{
				Move_Y = 0;
				Move_Z = 0;
			}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


