#include <aduc841.h>
#include "header.h"

void main(void) 
	{
   Init_Timer2();
    Init_Button();
    Timer3_Init();
    Init_UART();
    while(1) ;
    
}