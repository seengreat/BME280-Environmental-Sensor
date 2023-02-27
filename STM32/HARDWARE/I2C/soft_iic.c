#include "soft_iic.h"
#include "delay.h"


void IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;// GPIO_Mode_Out_OD
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void IIC_Start(void)
{ 
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	usr_delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	usr_delay_us(4);
	IIC_SCL=0;
}

void IIC_Stop(void)
{	
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	usr_delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;
	usr_delay_us(4);	
}

void IIC_Ack(void)
{	
	IIC_SCL=0;
	IIC_SDA=0;
	usr_delay_us(1);
	IIC_SCL=1;
	usr_delay_us(1);
	IIC_SCL=0;
	usr_delay_us(1);
}

void IIC_NAck(void)
{	
	IIC_SCL=0;
	IIC_SDA=1;
	usr_delay_us(2);
	IIC_SCL=1;
	usr_delay_us(2);
	IIC_SCL=0;
}

u8 IIC_Wait_Ack(void) 	 //return:=1 ASK, =0 noASK
{
	u8 ucErrTime=0;   
	IIC_SDA=1;
	usr_delay_us(1);	   
	IIC_SCL=1;
	usr_delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;
	return 0;	
}

void IIC_Send_Byte(u8 txd) //From the MSB to LSB
{
    u8 t=0;
	  IIC_SCL=0;
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
				usr_delay_us(2);   
				IIC_SCL=1;
				usr_delay_us(2); 
				IIC_SCL=0;	
				usr_delay_us(2);
    }
}  


u8 IIC_Read_Byte(u8 ack)  //From the MSB to LSB
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    IIC_SDA = 1;				
    while(i--)
    {
      ReceiveByte<<=1;      
      IIC_SCL = 0;
      usr_delay_us(1);
			IIC_SCL = 1;
      usr_delay_us(1);	
      if(READ_SDA)
      {
        ReceiveByte|=0x01;
      }
    }
    IIC_SCL = 0;

	if (ack)
		IIC_Ack();
	else
		IIC_NAck();  
  return ReceiveByte;

} 







