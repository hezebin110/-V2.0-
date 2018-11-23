#include "Driver_DBUS.h"
#include "Motor_Shoot.h"

//遥控需要此部分程序
#define Speed_Coefficient 8   //遥控速度系数
#define Left_Coefficient  5   //左转系数
#define Right_Coefficient 8   //右转系数

//键盘控制系数
#define KEY_Coefficient 2


#define IS ==

RC_Ctl_t RC_Ctl;
int16_t buff_zero[4], buff_one[4], buff_two[4], buff_thr[4], moto_ctr[6];

//double mouse_206,mouse_205;
//uint8_t mouse_flag;
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
extern int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 



//遥控解码函数
//在接收完毕后自动调用
//调用文件: Driver_uart.c
void Yaokong_translate(void)
{
    RC_Ctl.rc.ch0 = (uart1_rx_buff[0]| (uart1_rx_buff[1] << 8)) & 0x07ff;	//!< Channel 0通道
    RC_Ctl.rc.ch1 = ((uart1_rx_buff[1] >> 3) | (uart1_rx_buff[2] << 5)) & 0x07ff;	//!< Channel 1
    RC_Ctl.rc.ch2 = (((int16_t)uart1_rx_buff[2] >> 6) | ((int16_t)uart1_rx_buff[3] << 2) | ((int16_t)uart1_rx_buff[4] << 10)) & 0x07ff;	//!< Channel 2
    RC_Ctl.rc.ch3 = ((uart1_rx_buff[4] >> 1) | (uart1_rx_buff[5] << 7)) & 0x07ff;	//!< Channel 3
    RC_Ctl.rc.s1	= ((uart1_rx_buff[5] >> 4)& 0x000C) >> 2;	//!< Switch left  遥控器发射位机 S1开关位
    RC_Ctl.rc.s2	= ((uart1_rx_buff[5] >> 4)& 0x0003);	//!< Switch right
    RC_Ctl.mouse.x = uart1_rx_buff[6] | (uart1_rx_buff[7] << 8); //!< Mouse X axis  
    RC_Ctl.mouse.y = uart1_rx_buff[8] | (uart1_rx_buff[9] << 8); //!< Mouse Y axis
    RC_Ctl.mouse.z = uart1_rx_buff[10] | (uart1_rx_buff[11] << 8);  //!< Mouse Z axis
    RC_Ctl.mouse.press_l = uart1_rx_buff[12]; //!< Mouse Left Is Press ? 
    RC_Ctl.mouse.press_r = uart1_rx_buff[13]; //!< Mouse Right Is Press ?
    RC_Ctl.key.v=uart1_rx_buff[14]|uart1_rx_buff[15]<<8;
}

//清空遥控器数据
void DBUS_Init()
{
	      RC_Ctl.rc.ch0 = 1024;
				RC_Ctl.rc.ch1 = 1024;
				RC_Ctl.rc.ch2 = 1024;
      	RC_Ctl.rc.ch3 = 1024;
	
	
				RC_Ctl.rc.s1	= 0;
				RC_Ctl.rc.s2	= 0;
	
	
				RC_Ctl.mouse.x =0;
				RC_Ctl.mouse.y = 0;
				RC_Ctl.mouse.z = 0;
	
	
				RC_Ctl.mouse.press_l =0; 
				RC_Ctl.mouse.press_r =0;
	
	
	            RC_Ctl.key.v=0;
}

/******************************************************
                      底盘电机控制2017(上一届)

1, 就算出四种姿态的输出
2，四种姿态的叠加
3，准备CAN的数据输出
4，开始电机控制
__________________________________________________________________
| 电机位置 |电机号| 输出\方向 | 前 | 后 | 左 | 右 | 顺 | 逆 | 停 | 
``````````````````````````````````````````````````````````````````
|   左前   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      
``````````````````````````````````````````````````````````````````
|   右前   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|   左后   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|   右后   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |
``````````````````````````````````````````````````````````````````
|  备注:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
//遥控控制
const int THRESHOLD=2000;  //转弯阈值
const int TURNSPEED=2000;   //转弯极限速度

//键盘控制
#define SPEED_S       0
#define SPEED_NORMAL   3000     //正常前进速度

//组合按键的情况下斜着跑  3000:1500 22.5度向前   3000:0    45度        3000：-3000旋转  
#define SPEED_Y        3500     //前行向速度
#define SPEED_X        3000      //横向速度


//前  >0    <0  	<0  	>0
//左平移    <0    <0  	>0 	  >0
//左前 -           - 

//右平移  >0    >0  	<0 	  <0
//后  <0    >0		>0		<0
//左  <0    <0  	>0 	  >0
//            -          -

//前-转弯 速度合成
#define SPEED_T1        3500    //拐弯速度1
#define SPEED_T2        1500    //拐弯速度2

//中速原地转弯
#define SPEED_T3        1500        //原地转弯速度1
#define SPEED_T4        2000        //原地转弯速度2

#define ADD_SPEED  1000


int16_t ADDTarget(int16_t last_target,int16_t real_target);


static void OneKeyDeal_WSADQR(uint16_t key);
static void TowKeyDeal_WSAD(uint16_t key);
static void  FastMode_Shift(int16_t *ctr);

//遥控的各个通道数据处理
void DBUS_Deal(void)
{
    //遥控器各通道数据处理
	buff_zero[0]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient; //*8  // 左右移动
	buff_zero[1]=((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[2]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	buff_zero[3]=-((int16_t)RC_Ctl.rc.ch0-1024)*Speed_Coefficient;
	
	buff_one[0]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;    // 前后移动  官方版本（官方和我们自己的车前后控制反向）
	buff_one[1]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	buff_one[2]=-((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	buff_one[3]=((int16_t)RC_Ctl.rc.ch1-1024)*Speed_Coefficient;
	
	buff_two[0]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;    //顺时针旋转
	buff_two[1]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	buff_two[2]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	buff_two[3]=((int16_t)RC_Ctl.rc.ch2-1024)*Left_Coefficient;
	
	
	buff_thr[0]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;   //逆时针旋转
	buff_thr[1]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
	buff_thr[2]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
	buff_thr[3]=-((int16_t)RC_Ctl.rc.ch3-1024)*Right_Coefficient;
 	
	
		//速度赋值
		ChassisPidLF.target=buff_zero[0]+buff_one[0]+buff_two[0]+buff_thr[0];
		ChassisPidRF.target=buff_zero[1]+buff_one[1]+buff_two[1]+buff_thr[1];
		ChassisPidRB.target=buff_zero[2]+buff_one[2]+buff_two[2]+buff_thr[2];
		ChassisPidLB.target=buff_zero[3]+buff_one[3]+buff_two[3]+buff_thr[3];
		
		
		if((ChassisPidLF.target/*目标*/>THRESHOLD /*2000*/)&&(ChassisPidRF.target>THRESHOLD)&&(ChassisPidRB.target>THRESHOLD)&&(ChassisPidLB.target>THRESHOLD))			//顺转速度控制
		{
			ChassisPidLF.target=TURNSPEED;   //2000
			ChassisPidRF.target=TURNSPEED;
			ChassisPidRB.target=TURNSPEED;
			ChassisPidLB.target=TURNSPEED;
		}
		
		if((ChassisPidLF.target<-THRESHOLD)&&(ChassisPidRF.target<-THRESHOLD)&&(ChassisPidRB.target<-THRESHOLD)&&(ChassisPidLB.target<-THRESHOLD))			//顺转速度控制
		{
	        ChassisPidLF.target=-TURNSPEED;
			ChassisPidRF.target=-TURNSPEED;
			ChassisPidRB.target=-TURNSPEED;
			ChassisPidLB.target=-TURNSPEED;
		}	  
}


//处理单个按键
static void OneKeyDeal_WSADQR(uint16_t KeyPress)
{
	
	if(KeyPress&KEY_PRESSED_W)    //when "W" is pushed ,then go along. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
		{
			moto_ctr[0]=SPEED_NORMAL;  //SPEED_NORMAL   3000     //正常前进速度
			moto_ctr[1]=-SPEED_NORMAL;
			moto_ctr[2]=-SPEED_NORMAL;
			moto_ctr[3]=SPEED_NORMAL;	
		}
		
	  if(KeyPress&KEY_PRESSED_S)    //when "S"  is pushed ,then go back. CAUTION:Our cars are different with DJIs` ,"Go along" and "Go back" is opposite.
		{
			moto_ctr[0]=-SPEED_NORMAL;
			moto_ctr[1]=SPEED_NORMAL;
			moto_ctr[2]=SPEED_NORMAL;
			moto_ctr[3]=-SPEED_NORMAL;			
		}
		
	   if(KeyPress&KEY_PRESSED_A)    //when "A" is pushed ,then rotate anticlockwise
		{
	    moto_ctr[0]=-SPEED_X;  // SPEED_X        3000      横向速度   
			moto_ctr[1]=-SPEED_X;
			moto_ctr[2]=SPEED_X;
			moto_ctr[3]=SPEED_X;		
		}
		
		if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then rotate clockwise
		{
			
			moto_ctr[0]=SPEED_X;
			moto_ctr[1]=SPEED_X;
			moto_ctr[2]=-SPEED_X;
			moto_ctr[3]=-SPEED_X;
	
		}
		
		if(KeyPress&KEY_PRESSED_Q)    //when "Q"  is pushed ,then go left
		{
			

			moto_ctr[0]=-SPEED_T3;   //SPEED_T3        1500        //原地转弯速度1
			moto_ctr[1]=-SPEED_T3;
			moto_ctr[2]=-SPEED_T3;
			moto_ctr[3]=-SPEED_T3;
		}
		
		if(KeyPress&KEY_PRESSED_R)    //when "R" is pushed ,then go right
		{
			moto_ctr[0]=SPEED_T3;
			moto_ctr[1]=SPEED_T3;
			moto_ctr[2]=SPEED_T3;
			moto_ctr[3]=SPEED_T3;
		}
		
		if((RC_Ctl.key.v==0)&&(RC_Ctl.mouse.x==0))    //when mouse is not moved and no key is pushed ,then stop
		{
			moto_ctr[0]=0;
			moto_ctr[1]=0;
			moto_ctr[2]=0;
			moto_ctr[3]=0;
		}
		
}


//速度合成
//前方    :  +  -  -  + 
//左旋    :  -  -  -  -
//左平    :  -  -  +  +
//前左旋  :  0        0


//前  >0    <0  	<0  	>0
//左平移    <0    <0  	>0 	  >0
//左前 -           - 

//右平移  >0    >0  	<0 	  <0
//后  <0    >0		>0		<0
//左  <0    <0  	>0 	  >0

//处理两个按键同时按下  斜向运动
static void TowKeyDeal_WSAD_T(uint16_t KeyPress)
{
	  //WA  左前方
		if((KeyPress&0X05)==0X05)    //when "W" and "A" are pushed ,then turn left
		{
			moto_ctr[0]=SPEED_T2;
			moto_ctr[1]=-SPEED_T1;
			moto_ctr[2]=-SPEED_T1;
			moto_ctr[3]=SPEED_T2;
		}
		
		//WD   右前方
		if((KeyPress&0X09)==0X09)    //when "W" and "D" are pushed ,then turn right
		{
			moto_ctr[0]=SPEED_T1;   
			moto_ctr[1]=-SPEED_T2;
			moto_ctr[2]=-SPEED_T2;
			moto_ctr[3]=SPEED_T1;
		}
		
		//WSAD   左后方  2,4减小
		if((KeyPress&0X06)==0X06)    //when "S" and "A" are pushed ,then turn left
		{
			moto_ctr[0]=-SPEED_T2;
			moto_ctr[1]=SPEED_T1;
			moto_ctr[2]=SPEED_T1;
			moto_ctr[3]=-SPEED_T2;
		}
		
		//SD    右后方  2,4减小
		if((KeyPress&0X0A)==0X0A)    //when S" and "D" are pushed ,then turn left
		{
			moto_ctr[0]=-SPEED_T1;
			moto_ctr[1]=SPEED_T2;
			moto_ctr[2]=SPEED_T2;
			moto_ctr[3]=-SPEED_T1;
		}

    if(KeyPress&0x0010)  //如果shift按下
		 {
			 if(RC_Ctl.key.v&0X0003)        //WS有按下的按键
		      FastMode_Shift(moto_ctr);
       else if(RC_Ctl.key.v&0X000C)    //AD按下
			    {
							if(KeyPress&KEY_PRESSED_A)    //when "A"  is pushed ,then go left
								{
									moto_ctr[0]=-SPEED_T3;
									moto_ctr[1]=-SPEED_T3;
									moto_ctr[2]=-SPEED_T3;
									moto_ctr[3]=-SPEED_T3;
								}
								
						 		if(KeyPress&KEY_PRESSED_D)    //when "D" is pushed ,then go right
								{
									moto_ctr[0]=SPEED_T3;
									moto_ctr[1]=SPEED_T3;
									moto_ctr[2]=SPEED_T3;
									moto_ctr[3]=SPEED_T3;
								}							
					}
       else
 			 {
				  	moto_ctr[0]=0;
						moto_ctr[1]=0;
						moto_ctr[2]=0;
				    moto_ctr[3]=0;
						
			 }				 
		 }
    		
}



////处理两个按键同时按下  斜向运动
//static void TowKeyDeal_WSAD_X(uint16_t KeyPress)
//{
//	  //WA  左前方
//		if((KeyPress&0X05)==0X05)    //when "W" and "A" are pushed ,then turn left
//		{
//			moto_ctr[0]=SPEED_X;
//			moto_ctr[1]=-SPEED_Y;
//			moto_ctr[2]=-SPEED_X;
//			moto_ctr[3]=SPEED_Y;
//		}
//		
//		//WD   右前方
//		if((KeyPress&0X09)==0X09)    //when "W" and "D" are pushed ,then turn right
//		{
//			moto_ctr[0]=SPEED_Y;   
//			moto_ctr[1]=-SPEED_X;
//			moto_ctr[2]=-SPEED_Y;
//			moto_ctr[3]=SPEED_X;
//		}
//		
//		
//		//WSAD   左后方  2,4减小
//		if((KeyPress&0X06)==0X06)    //when "S" and "A" are pushed ,then turn left
//		{
//			moto_ctr[0]=-SPEED_Y;
//			moto_ctr[1]=SPEED_X;
//			moto_ctr[2]=SPEED_Y;
//			moto_ctr[3]=-SPEED_X;
//		}
//		
//		//SD    右后方  2,4减小
//		if((KeyPress&0X0A)==0X0A)    //when S" and "D" are pushed ,then turn left
//		{
//			moto_ctr[0]=-SPEED_Y;
//			moto_ctr[1]=SPEED_X;
//			moto_ctr[2]=SPEED_Y;
//			moto_ctr[3]=-SPEED_X;
//		}	
//}



//遥控数据处理
void MotorTargetChange(void)	
{
   static int8_t Left_press=0;
	if(RC_Ctl.rc.s1==0)  //遥控器模式
	{											
     DBUS_Deal();  //遥控各个通道处理
	}

 else if(RC_Ctl.rc.s1==1)   //use mouse and keyboard to control
	{
		
		OneKeyDeal_WSADQR(RC_Ctl.key.v);  //处理单个按键 WSAD QR
		TowKeyDeal_WSAD_T(RC_Ctl.key.v);//0
		 
      //对鼠标左键进行判断，每次按下，需要发弹的次数加1
//    if(RC_Ctl.mouse.press_l==1)
//    {
//         if(Left_press==0)
//         {
//            Left_press=1;
//   //         ShootNumber.Number++;
//         }           
//    }
//    else
//       Left_press=0;
    
    
		if((RC_Ctl.key.v==32)&&(RC_Ctl.mouse.x>0)&&(RC_Ctl.mouse.x<=127))    //when "ctr" is pushed and mouse is moved to right ,then rotate clockwise
		{
			moto_ctr[0]=1620;
			moto_ctr[1]=1620;
			moto_ctr[2]=1620;
			moto_ctr[3]=1620;
		}
		
		if((RC_Ctl.key.v==32)&&(RC_Ctl.mouse.x>30000)&&(RC_Ctl.mouse.x<=32639))    //when "ctr" is pushed and mouse is moved to left ,then rotate anticlockwise
		{
			moto_ctr[0]=-1620;
			moto_ctr[1]=-1620;
			moto_ctr[2]=-1620;
			moto_ctr[3]=-1620;
		}
    
    if((RC_Ctl.mouse.x>32500)&&(RC_Ctl.mouse.x<=32643))    //when mouse is moved to up ,then rotate clockwise
		{
     //   YawOPID.target = YawOPID.target + (32643-RC_Ctl.mouse.x) *5;
    //    YawOPID.target = (YawOPID.target>5500)?5500:YawOPID.target;
		}
		if((RC_Ctl.mouse.x>0)&&(RC_Ctl.mouse.x<=50))    //when mouse is moved to down ,then rotate anticlockwise
    {
    //    YawOPID.target = YawOPID.target - (RC_Ctl.mouse.x+3) *5;
    //    YawOPID.target = (YawOPID.target<3000)?3000:YawOPID.target;
    }
   
//    if((RC_Ctl.mouse.y>32500)&&(RC_Ctl.mouse.y<=32643))    //when mouse is moved to up ,then rotate clockwise
//    {
//        PitchOPID.target = PitchOPID.target + (32643-RC_Ctl.mouse.y) *5;
//        PitchOPID.target = (PitchOPID.target>4700)?4700:PitchOPID.target;
//    }
//    
//    if((RC_Ctl.mouse.y>0)&&(RC_Ctl.mouse.y<=50))    //when mouse is moved to down ,then rotate anticlockwise
//    {
//        PitchOPID.target = PitchOPID.target - (RC_Ctl.mouse.y+3) *5;
//        PitchOPID.target = (PitchOPID.target<3750)?3750:PitchOPID.target;
//		}
//		mouse_flag++;
//		if(mouse_flag==5)                                                      //3000-5500
//		{
//        mouse_flag = 0;
//        if((RC_Ctl.mouse.y>32717)&&(RC_Ctl.mouse.y<=32640))    //when mouse is moved to up ,then rotate clockwise
//				{
////            PitchOPID.target = PitchOPID.target - (32640-PitchOPID.target) *15;
////            PitchOPID.target = (PitchOPID.target<3750)?3750:PitchOPID.target;
////            aim_angle_206 = aim_angle_206 - (32635-RC_Ctl.mouse.y) * 0.2;
////            aim_angle_206 = (aim_angle_206<3750)?3750:aim_angle_206;
//				}
//				if((RC_Ctl.mouse.y>0)&&(RC_Ctl.mouse.y<=50))    //when mouse is moved to down ,then rotate anticlockwise
//				{
////            PitchOPID.target = PitchOPID.target + (PitchOPID.target) *0.2;
////            PitchOPID.target = (PitchOPID.target>4700)?4700:PitchOPID.target;
////						aim_angle_206 = aim_angle_206 + RC_Ctl.mouse.y * 0.2;
////						aim_angle_206 = (aim_angle_206>5025)?5025:aim_angle_206;
//				}
//		}

			 
		ChassisPidLF.target=ADDTarget(ChassisPidLF.target,moto_ctr[0]);
	  ChassisPidRF.target=ADDTarget(ChassisPidRF.target,moto_ctr[1]);
		ChassisPidRB.target=ADDTarget(ChassisPidRB.target,moto_ctr[2]);
		ChassisPidLB.target=ADDTarget(ChassisPidLB.target,moto_ctr[3]);
	
	}

}

int16_t ADDTargetHead(int16_t last_target,int16_t real_target)
{
	  if(abs(last_target-real_target)>150)   //如果上一次的目标值与此次目标值相反
		{
			  if(last_target<real_target)   //如果上一次的目标值为负
				{
					  return last_target+150;    //目标转速加100
				}
				else
				{
					  return last_target-150;
				}
		}
		else 
			return real_target;
}


//防止因为目标值突变而引起失控
//目标速度值逐渐变化
//输入值：上一次的速度值 目标速度值
//返回  ：发出的转数值
int16_t ADDTarget(int16_t last_target,int16_t real_target)
{
	  if(abs(last_target-real_target)>2000)   //如果上一次的目标值与此次目标值相反
		{
			  if(last_target<real_target)   //如果上一次的目标值为负
				{
					  return last_target+150;    //目标转速加100
				}
				else
				{
					  return last_target-150;
				}
		}
		else 
			return real_target;
}

//加速状态
static void FastMode_Shift(int16_t ctr[])
{
	  char i=0;
	for(i=0;i<4;i++)
	{
		 if(ctr[i]<-1500)
			 ctr[i]-=ADD_SPEED;
		 else if(ctr[i]>1500)
			 ctr[i]+=ADD_SPEED;
	}
}
	
	
/****************************************************************
 * 延时函数
 *
 *
*****************************************************************/

void delay_ms(unsigned int t)
{
	int i;
  int a=0;
	for( i=0;i<t;i++)
	{
		for(a=0;a<42000;a++)
		{};
		//int a=42000; //at 168MHz 42000 is ok
		//while(a--);
	}
}



