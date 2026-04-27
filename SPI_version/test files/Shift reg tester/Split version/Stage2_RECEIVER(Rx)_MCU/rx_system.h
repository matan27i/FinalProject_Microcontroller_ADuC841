/* =============================================================================
   File: rx_system.h
   Role: Master system header (SPI Slave reception)

   Every .c file in the project includes exactly this one header.
   =============================================================================
*/

#ifndef RX_SYSTEM_H
#define RX_SYSTEM_H

#include <aduc841.h>

/* ---------------------------------------------------------------------------
   Frame-sync interrupt pin
   The Mega's /SS line is connected to P3.2 (INT0) on the ADuC841.
   ---------------------------------------------------------------------------
*/
sbit SPI_FRAME_SYNC = P3^2;

/* ---------------------------------------------------------------------------
   Module headers -- include order follows the dependency chain.
   ---------------------------------------------------------------------------
*/
#include "rx_types.h"
#include "rx_hw.h"
#include "rx_ecc.h"
#include "rx_decoder.h"

#endif /* RX_SYSTEM_H */
