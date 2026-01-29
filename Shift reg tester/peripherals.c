#include <aduc841.h>
#include "header.h"

sbit SR_CLK   = P3^0;   
sbit SR_DATA  = P3^1;   
sbit BUTTON   = P3^2;   
sbit SR_LATCH = P3^3;   

volatile unsigned int counter = 0; 

void Init_Timer2(void) {
    SR_CLK = 0;
    SR_DATA = 1;  // Constant '1' input
    SR_LATCH = 0; 
	
    // 2. Configure Timer 2
    // T2CON = 0x00: 16-bit Auto-Reload mode
    T2CON = 0x00;     
    RCAP2H = 0xFC; 
    RCAP2L = 0x66;     
    ET2 = 1;  
    EA = 1;   
    TR2 = 0;  
}

void Init_Button(void) {    
    IT0 = 1; 
    EX0 = 1; 
}



void External0_ISR(void) interrupt 0 {
    counter = 0;    
    TH2 = 0xFC;
    TL2 = 0x66;    
    TR2 = 1; 
}

void Timer2_ISR(void) interrupt 5 {
    TF2 = 0; 
    
    if(counter < 2) {
        // Toggle Clock and Latch
        SR_CLK = ~SR_CLK; 
        SR_LATCH = ~SR_LATCH;
        counter++;
    }
    else {
        TR2 = 0; 
        SR_CLK = 0;
        SR_LATCH = 0;
    }
}