/**
  *@file Driver_uart.c
  *@date 2018-1-14
  *@author ��������
  *@brief 
  */

#include "Driver_usart.h"


uint8_t uart1_rx_buff[50];
//uint8_t uart2_rx_buff[15]; 
uint8_t uart3_rx_buff[50];
uint8_t uart6_rx_buff[50];

//extern uint16_t aim_angle_206;
//extern uint16_t aim_angle_205;
extern uint8_t start_flag;
extern PidTypeDef ChassisPidLF,ChassisPidRF,ChassisPidRB,ChassisPidLB;
//extern int16_t ANGLE;


//it will be auto callback when usart receive msg completely
//ң�ؽ���18�����ݽ������жϺ���
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)           //������ɻص�����
{
  if(huart == &huart6)
  {

//		HAL_UART_Transmit(huart, uart6_rx_buff, 18, 100);
   
		__HAL_UART_CLEAR_PEFLAG(&huart6);                             //???
    HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 18);
	}
	
	
  if(huart == &huart1)
  {
		//ң��������֡������
			Yaokong_translate();
			start_flag = 1;
		//ֱ�ӷ���ֵ
//		HAL_UART_Transmit(huart, uart1_rx_buff, 18, 100);
		//��usart6����ֵ
//		HAL_UART_Transmit(&huart6, uart1_rx_buff, 15, 100); 
	
		__HAL_UART_CLEAR_PEFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18);
	}
	
	//�������̿�
//	if(huart == &huart2)
//  {
//     ANGLE=(uart2_rx_buff[2]-'0')*1000+(uart2_rx_buff[3]-'0')*100+(uart2_rx_buff[4]-'0')*10+(uart2_rx_buff[6]-'0');   
//	   __HAL_UART_CLEAR_PEFLAG(&huart2);
//	}
	
	
	
}


void UART_SendByte (unsigned char dat)
{ 
	while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	USART6->DR = (uint8_t) dat; 
}


void uart_putbuff(uint8_t *buff, unsigned int len)
{
    while(len--)
    {
        UART_SendByte (*buff);
        buff++;
    }
}


//extern uint16_t RealAnglePITCH,RealAngleYAW;
//extern int16_t RealAnglePLUCK,RealSpeedPLUCK;

//extern IMUDataTypedef imu_data;

//void VcanSendHead(void)  // ��ӡ��̨���ݵ�
//{
//	 uint8_t sendnum[16]={0};
//	 static char i=0;
	 
//	 sendnum[0]=(uint8_t)((RealAnglePITCH)&0XFF);
//	 sendnum[1]=(uint8_t)((RealAnglePITCH>>8)&0XFF);
	 
	
//	 sendnum[4]=(uint8_t)((RealAngleYAW)&0XFF);
//	 sendnum[5]=(uint8_t)((RealAngleYAW>>8)&0XFF);
	 
//	 sendnum[6]=(uint8_t)((ElectricYaw205)&0XFF);
//	 sendnum[7]=(uint8_t)((ElectricYaw205>>8)&0XFF);	 
	 
//	 sendnum[4]=(uint8_t)((RealAngleYAW)&0XFF);
//	 sendnum[5]=(uint8_t)((RealAngleYAW>>8)&0XFF);
	 

	 
	 //����Ŀ��ת��ֵ
//	 sendnum[8]=(uint8_t)((imu_data.az)&0XFF);
//	 sendnum[9]=(uint8_t)((imu_data.az>>8)&0XFF);
	 
//	 sendnum[10]=(uint8_t)((imu_data.gx)&0XFF);
	// sendnum[11]=(uint8_t)((imu_data.gx>>8)&0XFF);
	 
//	 sendnum[12]=(uint8_t)((imu_data.gy)&0XFF);
//	 sendnum[13]=(uint8_t)((imu_data.gy>>8)&0XFF);
  
//	 sendnum[14]=(uint8_t)((imu_data.gz)&0XFF);
//	 sendnum[15]=(uint8_t)((imu_data.gz>>8)&0XFF);

   // imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
   
//	  vcan_sendware(sendnum,16);
// }

//�����ٶ�ֵ�����ֵ,���䵽ɽ����λ����ʾ
void VcanSendSpeedElectric(void)
{
	 uint8_t sendnum[16]={0};

	 //����ת��ֵ	 
	 sendnum[0]=(uint8_t)((RealSpeedLF)&0XFF);
	 sendnum[1]=(uint8_t)((RealSpeedLF>>8)&0XFF);
	 
	 sendnum[2]=(uint8_t)((RealSpeedRF)&0XFF);
	 sendnum[3]=(uint8_t)((RealSpeedRF>>8)&0XFF);
	 
	 sendnum[4]=(uint8_t)((RealSpeedRB)&0XFF);
	 sendnum[5]=(uint8_t)((RealSpeedRB>>8)&0XFF);
	 
	 sendnum[6]=(uint8_t)((RealSpeedLB)&0XFF);
	 sendnum[7]=(uint8_t)((RealSpeedLB>>8)&0XFF);
	 
	 //����Ŀ��ת��ֵ
	 sendnum[8]=(uint8_t)((ChassisPidLF.target)&0XFF);
	 sendnum[9]=(uint8_t)((ChassisPidLF.target>>8)&0XFF);
	 
	 sendnum[10]=(uint8_t)((ChassisPidRF.target)&0XFF);
	 sendnum[11]=(uint8_t)((ChassisPidRF.target>>8)&0XFF);
	 
	 sendnum[12]=(uint8_t)((ChassisPidRB.target)&0XFF);
	 sendnum[13]=(uint8_t)((ChassisPidRB.target>>8)&0XFF);
	 	 vcan_sendware(sendnum,16);	
}

//  if(i==10)
//  {
//		 i=0;
//		 sendnum[14]=(uint8_t)((10000)&0XFF);
//  	 sendnum[15]=(uint8_t)((10000>>8)&0XFF);
//	}
//  else
//	{
//		i++;
// 		 sendnum[14]=(uint8_t)((-10000)&0XFF);
//  	 sendnum[15]=(uint8_t)((-10000>>8)&0XFF);
//  
//	}		
	 //sendnum[14]=(uint8_t)((ChassisPidLB.target)&0XFF);
	 //sendnum[15]=(uint8_t)((ChassisPidLB.target>>8)&0XFF);
	 
	 
	 //���͵���ֵ 
//	 sendnum[8]=(uint8_t)((ElectricOutLF)&0XFF);
//	 sendnum[9]=(uint8_t)((ElectricOutLF>>8)&0XFF);
//	 
//	 sendnum[10]=(uint8_t)((ElectricOutRF)&0XFF);
//	 sendnum[11]=(uint8_t)((ElectricOutRF>>8)&0XFF);
//	 
//	 sendnum[12]=(uint8_t)((ElectricOutRB)&0XFF);
//	 sendnum[13]=(uint8_t)((ElectricOutRB>>8)&0XFF);

//	 sendnum[14]=(uint8_t)((ElectricOutLB)&0XFF);
//	 sendnum[15]=(uint8_t)((ElectricOutLB>>8)&0XFF);
//	 



/**************************************************************
*�������ݵ���λ������ʾ���� ɽ����λ����������
* wareaddr: ��Ҫ���͵������ַ,Ĭ��uint8,ÿһ�����ݴ���һ��ͨ��,���Ϊ8
* waresize: �������ݵĴ�С
*
***************************************************************/
void vcan_sendware(uint8_t *wareaddr, unsigned int waresize)
{
    #define CMD_WARE     3   //ʹ������ʾ����ģʽ
    unsigned char cmdf[2] = {CMD_WARE, ~CMD_WARE};  
    unsigned char cmdr[2] = {~CMD_WARE, CMD_WARE};    

    uart_putbuff(cmdf, sizeof(cmdf));    //����֡ͷ
	uart_putbuff(wareaddr, waresize);   //��������
     
    uart_putbuff(cmdr, sizeof(cmdr));    
}
