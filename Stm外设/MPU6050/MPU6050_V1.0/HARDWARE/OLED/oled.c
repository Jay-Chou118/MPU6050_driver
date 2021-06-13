#include "oled.h"
#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"

u8 OLED_GRAM[128][8];	 

u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}


//dat:要写入的数据/命令
//cmd：0表示命令，1表示数据
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	u8 i;
	
	if(cmd)
		OLED_DC_Set();
	else
		OLED_DC_Clr();   
	
	OLED_CS_Clr();  //拉低CS
	
	for(i = 0; i < 8; i++)
	{
		OLED_SCLK_Clr();
		if(dat & 0x80)
			OLED_SDIN_Set();
		else
			OLED_SDIN_Clr();
		OLED_SCLK_Set();
		
		dat <<= 1;
	}
	
	OLED_CS_Set();
}

void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	

void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//更新显示
}

//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//得到偏移后的值				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=asc2_1206[chr][t];  //调用1206字体
		else temp=asc2_1608[chr][t];		 //调用1608字体 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
//用16字体
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   

/***********************************************************/
//x:表示的列，分为（列低地址和列高地址）， y:表示的是page页编号(0~7)
void OLED_Set_Pos(u8 x, u8 y)
{
	OLED_WR_Byte(0xb0 + y, OLED_CMD);
	
	OLED_WR_Byte((x&0x0F), OLED_CMD);
	OLED_WR_Byte((x&0xF0)>>4 | 0x10, OLED_CMD);
}

//y0~y1的取值范围是0~7，x0~x1的取值范围是0~127
void OLED_DrawBMP(u8 x0, u8 y0, u8 x1, u8 y1, u8 bmp[])
{
	unsigned int i = 0;
	unsigned char x, y;
	
	for(y = y0; y <= y1; y++)
	{
		OLED_Set_Pos(x0, y);
		for(x = x0; x <= x1; x++)
		{
			OLED_WR_Byte(bmp[i++], OLED_DATA);
		}
	}
}
/***********************************************************/
//补充部分如上。
/***********************************************************/
void OLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
	
	OLED_RST_Set();
	delay_us(100000);
	OLED_RST_Clr();
	delay_us(20);
	OLED_RST_Set();
	
	OLED_WR_Byte(0xA8, OLED_CMD);
	OLED_WR_Byte(0x3F, OLED_CMD);
	OLED_WR_Byte(0xD3, OLED_CMD);
	OLED_WR_Byte(0x00, OLED_CMD);
	OLED_WR_Byte(0x40, OLED_CMD);
	OLED_WR_Byte(0xA1, OLED_CMD);   //A0/A1:控制屏幕显示左右映射翻转的
	OLED_WR_Byte(0xC0, OLED_CMD);   //C0/C8：控制屏幕上下映射翻转的
	OLED_WR_Byte(0xDA, OLED_CMD);
	OLED_WR_Byte(0x12, OLED_CMD);   //02h:隔行显示  12h:逐行显示
	OLED_WR_Byte(0x81, OLED_CMD);
	OLED_WR_Byte(0x7F, OLED_CMD);
	OLED_WR_Byte(0xA4, OLED_CMD);
	OLED_WR_Byte(0xA6, OLED_CMD);
	OLED_WR_Byte(0xD5, OLED_CMD);
	OLED_WR_Byte(0x80, OLED_CMD);
	OLED_WR_Byte(0x8D, OLED_CMD);
	OLED_WR_Byte(0x14, OLED_CMD);
	OLED_WR_Byte(0xAF, OLED_CMD);
	
	OLED_Clear();
}
