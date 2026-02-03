#include <aduc841.h>
#include "header.h"

sbit SR_CLK   = P2^0;
sbit SR_DATA  = P2^1;
sbit BUTTON   = P3^2;
sbit SR_LATCH = P2^3;
sbit SR_CLR   = P2^4;

volatile unsigned int counter = 0;
volatile unsigned int flag_c = 0;

/*void Delay_Debounce(void)
	{
    unsigned int i;
    for(i = 0; i < 20000; i++); 
	}
*/

void Init_Timer2(void) {
    SR_CLK = 0;
    SR_DATA = 1;
    SR_LATCH = 1;
    SR_CLR = 1;

    T2CON = 0x00;
    RCAP2H = 0xD4;
    RCAP2L = 0xCD;
    ET2 = 1;
    EA = 1;
    TR2 = 0;
}

void Init_Button(void) {
    BUTTON = 1;
    IT0 = 1;
    EX0 = 1;
}

void Timer3_Init(void) {
    T3CON &= 0xFE;
    T3CON |= 0x86;
    T3FD = 0x08;
}

void Init_UART(void) {
    SM0 = 0;
		SM1 = 1;
		REN = 1;
    RI = 0;
    TI = 0;
    ES = 1;
}

void External0_ISR(void) interrupt 0 {
    counter = 0;
    TH2 = 0xD4;
    TL2 = 0xCD;
    TR2 = 1;
}

void UART_ISR(void) interrupt 4 {
    if (RI) {
        unsigned char received_char;
        RI = 0;
        received_char = SBUF;

        if (received_char == 'A' ) 
					{
            counter = 0;
            TH2 = 0xD4;
            TL2 = 0xCD;
            TR2 = 1;
					}

        if (received_char == 'C') 
					{
            flag_c = 1;
            counter = 0;
            TH2 = 0xD4;
            TL2 = 0xCD;
            TR2 = 1;
        }
    }
    if (TI) {
        TI = 0;
    }
}

void Timer2_ISR(void) interrupt 5 
	{
   TF2 = 0;
	 if (flag_c != 1)
	 {
			if(counter < 2) 
				{
					SR_LATCH = SR_CLK;
					SR_CLK = ~SR_CLK;
					counter++;
				}
			else
				{
				TR2 = 0;
				SR_CLR = 1;
				}
		}
		else
		{
			if(counter < 4) 
			{
				SR_CLR = 0;
				SR_LATCH = SR_CLK;
				SR_CLK = ~SR_CLK;
				counter++;
			}
			else
				{
				TR2 = 0;
				SR_CLR = 1;
				flag_c = 0;
				}
		}
	 }
	