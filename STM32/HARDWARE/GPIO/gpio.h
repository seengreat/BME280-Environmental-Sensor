#ifndef __GPIO_H
#define __GPIO_H
#include "sys.h"

#define	NSS PBout(12)  
#define	LED PBout(4) 


void IO_Init(void);

#endif

