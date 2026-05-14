#include <aduc841.h>

/*
 * Drive P0.0 / P0.1 / P0.2 with one of four patterns selected over the
 * UART.  Send the ASCII digit '0', '1', '2', or '3' to pick the pattern;
 * outputs hold until the next selection.  Other characters are ignored.
 */

static const unsigned char patterns[4] = {
    0x02,  /* P0.0=0  P0.1=1  P0.2=0 */
    0x05,  /* P0.0=1  P0.1=0  P0.2=1 */
    0x07,  /* P0.0=1  P0.1=1  P0.2=1 */
    0x00   /* P0.0=0  P0.1=0  P0.2=0 */
};

void UART_Init(void)
{
    /* Timer 1 mode 2 (8-bit auto-reload), 9600 baud @ 11.0592 MHz */
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1   = 0xDC;
    TL1   = 0xDC;
    TR1   = 1;

    SCON  = 0x50;   /* mode 1 (8-bit async), REN = 1 */
}

unsigned char UART_Read(void)
{
    while (!RI);
    RI = 0;
    return SBUF;
}

void main(void)
{
    unsigned char ch;

    /* CD=0 - keep the core at full 11.0592 MHz so the baud rate is correct. */
    PLLCON = 0x00;

    UART_Init();
    
    /* Initialize Port 0 */
    P0 = 0x00;

    while (1)
    {
        ch = UART_Read();
        if (ch >= '0' && ch <= '3')
        {
            /* Drive Port 0 based on the selected pattern */
            P0 = patterns[ch - '0'];
        }
    }
}