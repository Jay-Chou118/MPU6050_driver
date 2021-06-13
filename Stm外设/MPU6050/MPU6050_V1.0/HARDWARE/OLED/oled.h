/*
	OLED0.96                  							STM32
	GND        				<----------->					GND
	VCC        				<----------->					3.3V
	D0(SCLK)         	<----------->					PA0			
	D1SDIN()         	<----------->					PA1
	RES        				<----------->					PA2
	DC(Data/Command)	<----------->					PA3
	CS								<----------->					PA4
*/


#ifndef __OLED_H__
#define __OLED_H__

#include "sys.h"

//OLED端口配置
#define OLED_SCLK_Set()	GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define OLED_SCLK_Clr()	GPIO_ResetBits(GPIOA, GPIO_Pin_0)

#define OLED_SDIN_Set()	GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define OLED_SDIN_Clr()	GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define OLED_RST_Set() GPIO_SetBits(GPIOA, GPIO_Pin_2)
#define OLED_RST_Clr() GPIO_ResetBits(GPIOA, GPIO_Pin_2)

#define OLED_DC_Set()		GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define OLED_DC_Clr()		GPIO_ResetBits(GPIOA, GPIO_Pin_3)

#define OLED_CS_Set() 	GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define OLED_CS_Clr() 	GPIO_ResetBits(GPIOA, GPIO_Pin_4)


#define OLED_CMD	0 //写命令
#define OLED_DATA	1 //写数据


//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	


/***********************************************************/
void OLED_Set_Pos(u8 x, u8 y);
void OLED_DrawBMP(u8 x0, u8 y0, u8 x1, u8 y1, u8 bmp[]);
/***********************************************************/
	

#endif
