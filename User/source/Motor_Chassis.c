
//���̵���ջ����Ƴ���
//��������
//2018/1/12
//

#include "Motor_Chassis.h"
#include <stdio.h>  


PidTypeDef ChassisPidLF={0},ChassisPidRF={0},ChassisPidRB={0},ChassisPidLB={0};  //
//PidTypeDef YawOPID={0};   //��ƫ��        { 2000, 1.6, 0.05, 10, {0,0,0,0,0}, 0}; 


/**
  * @brief  PID��ʼ������
  * @param  void
  * @retval void
  */

const double KP=7,KI=0.3,KD=0;  //���̵��PID
int16_t ANGLE=0;

//extern int16_t SPEEDTARGET;
int16_t SPEEDTARGET=0;   //Ŀ���ٶ�
void PIDInit(void)
{
	//  char i=0;
	//���̵��LF  ��ǰ
	ChassisPidLF.Kp=KP;
    ChassisPidLF.Ki=KI;
	ChassisPidLF.Kd=KD;
	 // ChassisPidLF.AddMax=1500;
	ChassisPidLF.Istart=200;
	ChassisPidLF.OutMax=2000;
	ChassisPidLF.SumMax=20000;
  	ChassisPidLF.target=SPEEDTARGET; 

	
		//���̵��RF
	ChassisPidRF.Kp=KP;
    ChassisPidRF.Ki=KI;
	ChassisPidRF.Kd=KD;
	  ///ChassisPidRF.AddMax=1500;
	  ChassisPidRF.Istart=200;
	  ChassisPidRF.OutMax=2000;
	  ChassisPidRF.SumMax=20000;
  	ChassisPidRF.target=SPEEDTARGET; 

	
	//���̵��RB
	  ChassisPidRB.Kp=KP;
    ChassisPidRB.Ki=KI;
	  ChassisPidRB.Kd=KD;
		//ChassisPidRB.AddMax=1500;
		ChassisPidRB.Istart=200;
	  ChassisPidRB.OutMax=2000;
	  ChassisPidRB.SumMax=20000;
  	ChassisPidRB.target=SPEEDTARGET; 

			
//���̵��LB			
	  ChassisPidLB.Kp=KP;
    ChassisPidLB.Ki=KI;
	  ChassisPidLB.Kd=KD;
		//ChassisPidLB.AddMax=1500;
		ChassisPidLB.Istart=200;
	  ChassisPidLB.OutMax=2000;
	  ChassisPidLB.SumMax=20000;
  	ChassisPidLB.target=SPEEDTARGET; 	
	
}

int16_t ElectricOutLF,ElectricOutRF,ElectricOutRB,ElectricOutLB; 
extern uint8_t un_control;
/** 
  * @note   modified
  * @brief  ���̽Ƕ�PID
  * @param  
  * @retval void
  */
void Control_ChassisPID(void)
{
	ElectricOutLF = PIDCalc(&ChassisPidLF,RealSpeedLF);
	ElectricOutRF = PIDCalc(&ChassisPidRF,RealSpeedRF);
	ElectricOutRB = PIDCalc(&ChassisPidRB,RealSpeedRB);
	ElectricOutLB = PIDCalc(&ChassisPidLB,RealSpeedLB);

	//�����̵�����ٶȽ��뷢��
	MotorTxData[0] = (uint8_t)((ElectricOutLF>>8)&0xFF);
	MotorTxData[1] = (uint8_t)(ElectricOutLF&0xFF); 
	
	MotorTxData[2] = (uint8_t)((ElectricOutRF>>8)&0xFF);
	MotorTxData[3] = (uint8_t)(ElectricOutRF&0xFF); 
	
	MotorTxData[4] = (uint8_t)((ElectricOutRB>>8)&0xFF);
	MotorTxData[5] = (uint8_t)(ElectricOutRB&0xFF); 
	
	MotorTxData[6] = (uint8_t)((ElectricOutLB>>8)&0xFF);
	MotorTxData[7] = (uint8_t)(ElectricOutLB&0xFF); 

	  if(un_control>=12)    //0.3s������ת�����߻ظ�״̬1.2s��Ȼû�дﵽĿ���ٶ�0����ǿ�Ƶ���������Ϊ0
	{	   		
		for(char i=0;i<8;i++)
	       MotorTxData[i]=0; 		 			
	}
  	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
}





