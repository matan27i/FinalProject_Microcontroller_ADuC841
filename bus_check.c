#include <aduc841.h>


sbit TRIGGER = P3^4;  // measring pin command 
sbit BUTTON  = P3^2;  // switch button

(Delay) 
void delay(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++) {
        for(j = 0; j < 60; j++); //delay to catch the signal 
    }
}

void main(void) {
    unsigned char mode = 0; // start state
    P2 = 0x00;
    TRIGGER = 0;

    while(1) {
        if (BUTTON == 0) {
            delay(20); //  (Debounce)
            
            if (BUTTON == 0) {
                mode++; //next stage 
                if (mode > 2) mode = 1; // 2 checking state

               
                if (mode == 1) {
                    // 
                    TRIGGER = 1; 
                    P2 = 0x02;   // p2.1 on
                } 
                else if (mode == 2) {
                    // 
                    TRIGGER = 1; 
                    P2 = 0x05;   // p2.0 and p2.2 on
                }

                delay(100);       
                P2 = 0;           
                TRIGGER = 0;      

                while(BUTTON == 0); 
            }
        }
    }
}