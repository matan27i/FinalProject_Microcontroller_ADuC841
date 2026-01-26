/* File: tx_handler.c
 * UART Character Handler for H1-Type Stateful Bus Encoder
 *
 * INPUT FORMAT:
 * The host (external Python script) sends arbitrary 8-bit characters (ASCII).
 * Each character is split into TWO 4-bit nibbles:
 *   - HIGH nibble: bits 7..4 of the character
 *   - LOW nibble:  bits 3..0 of the character
 *
 * TRANSMISSION ORDER: HIGH nibble first, then LOW nibble.
 * Each nibble is processed as an independent S_new (4-bit syndrome).
 *
 * TERMINATOR HANDLING:
 * '\r' (0x0D) and '\n' (0x0A) are treated as batch terminators.
 * They set buffer_flag for compatibility with the original codebase.
 * They do NOT generate nibble transmissions.
 *
 */

#include <aduc841.h>
#include "header.h"

/* tx_handler
 * Processes a single character received from UART.
 *
 * For printable/data characters:
 * 1. Extract high nibble: (rx_char >> 4) & 0x0F
 * 2. Extract low nibble:  rx_char & 0x0F
 * 3. Process high nibble first via process_nibble()
 * 4. Process low nibble second via process_nibble()
 *
 * For terminators ('\r', '\n'):
 * - Set buffer_flag = 1 (for legacy batch processing compatibility)
 * - Do NOT process as nibbles
 *
 * The process_nibble() function handles the full encode cycle:
 * S_old computation, S_target = S_new ^ S_old, minimal-w search,
 * differential update, and shift register output.
 */
void tx_handler(uint8_t rx_char)
{
    uint8_t high_nibble;
    uint8_t low_nibble;
    
    /* --- Check for line terminators (batch boundary markers) --- */
    if (rx_char == '\r' || rx_char == '\n')
    {
        /* Set flag for main loop (preserved for compatibility) */
        buffer_flag = 1;
        return;
    }
    
    /* --- Process data character --- */
    
    /* Step 1: Extract high nibble (bits 7..4) */
    high_nibble = (rx_char >> 4) & 0x0F;
    
    /* Step 2: Extract low nibble (bits 3..0) */
    low_nibble = rx_char & 0x0F;
    
    /* Step 3: Process HIGH nibble FIRST (per specification) */
    process_nibble(high_nibble);
    
    /* Step 4: Process LOW nibble SECOND */
    process_nibble(low_nibble);
    
    /* Increment buffer count for tracking (optional, for debugging/stats) */
    if (buffer_count < 255)
    {
        buffer_count += 2;  /* Two nibbles processed */
    }
}
