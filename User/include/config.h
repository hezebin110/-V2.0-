// 配置头文件
//
//

       //目标速度

#define MAXELECTRICITY 2000      //底盘最大允许电流
#define YAWANGLEMAX   8000       //航偏角最大角度
#define YAWANGLEMIN   0          //航偏角最小角度
#define PITCHANGLEMAX   8000       //俯仰角最大角度
#define PITCHANGLEMIN   0          //俯仰角最小角度

#define INTEGARALSNUM    5        //PID积分次数

#define MOTORID   0x200  //底盘电机ID
#define HEADID    0x1FF   //云台电机ID  
 
 

#define CHEAKSTART     0X00  //校正开始
#define CHEAKING       0X40  //校正运行中
#define CHEAKFINISH    0X80  //校正结束


