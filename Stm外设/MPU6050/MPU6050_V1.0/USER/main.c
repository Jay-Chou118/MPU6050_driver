#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "oled.h"
#include "iic.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
 

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}  
 	

int main( void )
{
	
	short Accel[3];
	short Gyro[3];
	float Temp; 
	float pitch,roll,yaw; 		//欧拉角
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );       /* 设置NVIC中断分组2:2位抢占优先级，2位响应优先级 */
	uart_init( 500000 );                                      /* 串口初始化为500000 */
	delay_init();                                           /* 延时函数初始化 */
	LED_Init();
	OLED_Init();
	i2c_Init();																						//I2C初始化
	MPU6050_Init();  																			//MPU6050初始化

	while ( 1 )
	{
		if (MPU6050ReadID() == 1)	//检测MPU6050
	{		
		while(1)
		{
//			if(Task_Delay[0]==0)
//			{
//				Task_Delay[0]=1000;
//			}			
				OLED_Refresh_Gram();
				MPU6050ReadAcc(Accel);			
				//printf("\r\n加速度:%8d%8d%8d ",Accel[0],Accel[1],Accel[2]);
				MPU6050ReadGyro(Gyro);
				//printf("陀螺仪:%8d%8d%8d ",Gyro[0],Gyro[1],Gyro[2]);				
				MPU6050_ReturnTemp(&Temp); 
				//printf("温度:%8.2f",Temp);
				OLED_ShowNumber(65,4,Temp,3,16);
				delay_ms(2000);//更新一次数据，可根据自己的需求，提高采样频率，如100ms采样一次
				mpu6050_send_data(Accel[0],Accel[1],Accel[2],Gyro[0],Gyro[1],Gyro[2]);//用自定义帧发送加速度和陀螺仪原始数据
				usart1_report_imu(Accel[0],Accel[1],Accel[2],Gyro[0],Gyro[1],Gyro[2],(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			
	  }
  }	
}
}



