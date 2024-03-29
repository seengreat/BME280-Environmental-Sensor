#ifndef __SOFT_IIC_H
#define __SOFT_IIC_H
#include "sys.h"
   	   		   

#define IIC_SCL      PBout(10) //SCL
#define IIC_SDA      PBout(11) //SDA	 
#define READ_SDA     PBin(11) 

/*********************************************/

void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);	


#endif

