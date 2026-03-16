/* =============================================================================
   File: rx_system.h
   Role: Master system header (MODIFIED for SPI Slave reception)

   MODIFICATION SUMMARY:
     The original design read 25 parallel bits directly from Port pins
     (P0, P1, P2, P3.3, P3.4) with edge detection on P3.2 (DATA_READY).

     This version replaces the parallel bus interface with SPI slave
     reception from the Arduino Mega 2560 bridge.  The 25-bit word
     arrives as 4 SPI bytes, and the Mega's /SS line serves as both
     the SPI slave-select and the frame-sync signal (via INT0).

   HARDWARE CONNECTIONS (Arduino Mega 2560 -> ADuC841 RX):
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     Mega Pin 51 (MOSI)  -> ADuC841 MOSI  (SPI data in)
     Mega Pin 52 (SCK)   -> ADuC841 SCLOCK (SPI clock)
     Mega Pin 53 (SS)    -> ADuC841 /SS   (SPI slave select)
                         -> ADuC841 P3.2  (INT0, frame-sync edge)

   ADuC841 SPI PIN MAPPING:
   ~~~~~~~~~~~~~~~~~~~~~~~~
     The ADuC841 SPI peripheral uses dedicated pins that are active
     when SPE=1 in SPICON.  Refer to the ADuC841 datasheet (Table 10,
     "Pin Function Description") for the exact pin assignments on your
     package variant.  Common assignments:

       Function    Typical Pin     Notes
       ~~~~~~~~    ~~~~~~~~~~~~    ~~~~~~~~~~~~~~~~~~~~~~~~~~~
       SCLOCK      See datasheet   Clock input in slave mode
       MOSI        See datasheet   Data input in slave mode
       MISO        See datasheet   Data output (unused here)
       /SS         See datasheet   Active-low slave select

     IMPORTANT: The Mega's SS output must be connected to BOTH the
     ADuC841 /SS pin AND to P3.2 (INT0) for frame synchronisation.
     Use a simple wire fork or bus buffer if signal integrity requires it.

   UART (unchanged):
     P3.0 (RXD) -- not used (RX board is transmit-only to PC)
     P3.1 (TXD) -- UART output to PC

   Porting checklist:
     - Replace <aduc841.h> with your target MCU's device header.
     - Verify SPI pin assignments against your specific ADuC841 datasheet.
     - If using a different interrupt pin for /SS sync, update INT0
       references in rx_hw.c accordingly.
   =============================================================================
*/

#ifndef RX_SYSTEM_H
#define RX_SYSTEM_H

/* ---------------------------------------------------------------------------
   Target MCU device header
   Provides: SFR declarations (SPICON, SPIDAT, SPISTA, P3, SBUF,
             SCON bits, T3CON, T3FD, EA, TI, SM0, SM1, REN, ES),
             and the 'sbit'/'bit' keywords.
   ---------------------------------------------------------------------------
*/
#include <aduc841.h>

/* ---------------------------------------------------------------------------
   Frame-sync interrupt pin
   ---------------------------------------------------------------------------
   The Arduino Mega drives its /SS line low at the start of each 4-byte
   SPI frame.  This signal is connected to P3.2 (INT0) on the ADuC841.
   A falling-edge interrupt on INT0 resets the SPI byte counter to
   maintain frame alignment.

   If /SS de-bouncing is needed (unlikely with a direct MCU-to-MCU
   connection), add a small RC filter on this line.
   ---------------------------------------------------------------------------
*/
sbit SPI_FRAME_SYNC = P3^2;  /* INT0 pin, directly driven by Mega /SS */

/* ---------------------------------------------------------------------------
   Module headers -- include order follows the dependency chain:
     rx_types   (no deps)
     rx_hw      (uses rx_types, provides SPI + UART functions)
     rx_ecc     (uses rx_types, calls rx_decoder for H1 syndrome)
     rx_decoder (uses rx_types, calls rx_hw for UART output)
   ---------------------------------------------------------------------------
*/
#include "rx_types.h"
#include "rx_hw.h"
#include "rx_ecc.h"
#include "rx_decoder.h"

#endif /* RX_SYSTEM_H */
