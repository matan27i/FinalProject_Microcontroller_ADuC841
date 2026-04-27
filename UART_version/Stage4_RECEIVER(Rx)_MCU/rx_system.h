/*
   File: rx_system.h
   Role: Master system header (UART Frame reception)

   Every .c file in the project includes exactly this one header.

   REFACTOR NOTE:
     The receiver's ingress path was changed from SPI slave to UART
     framed input.  The ADuC841's SPI slave is vulnerable to clock
     glitches on long jumper wires -- a single SCLK spike permanently
     shifts the byte boundary for the rest of the session.  UART is
     self-resynchronising at every start bit, so transient noise on
     the bridge-to-RX link corrupts at most one byte.  Byte-level
     integrity is restored by a 6-byte framed protocol with a CRC-8.
*/

#ifndef RX_SYSTEM_H
#define RX_SYSTEM_H

#include <aduc841.h>

/*
   Module headers include order follows the dependency chain.
*/
#include "rx_types.h"
#include "rx_hw.h"
#include "rx_ecc.h"
#include "rx_decoder.h"

#endif
