#ifndef __IIC_H_
#define __IIC_H_

#include "sys.h"


#define I2C_WR	0		/* Ð´¿ØÖÆbit */
#define I2C_RD	1		/* ¶Á¿ØÖÆbit */

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(u8 _ucByte);
u8 i2c_ReadByte(u8 ack);
u8 i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
u8 i2c_CheckDevice(u8 _Address);
void i2c_Init(void);


#endif
