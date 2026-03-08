/* File: header.h
   Master header file that aggregates all modular headers.
   Include this file in all .c files to gain access to the full system.
*/

#ifndef HEADER_H
#define HEADER_H

/* Include common types and constants first */
#include "common_types.h"

/* Include module-specific headers */
#include "main.h"
#include "peripherals.h"
#include "bus_encoder.h"
#include "init_pesec_matrices.h"
#include "ecc.h"
#include "tx_handler.h"
#include "shift_output.h"

#endif /* HEADER_H */