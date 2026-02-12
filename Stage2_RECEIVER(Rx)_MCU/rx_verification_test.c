/* File: rx_verification_test.c
   
   Syndrome Computation Verification Test
   Purpose: Verify RX syndrome matches TX syndrome for all test vectors
   
   This test can be compiled and run on a PC to verify the algorithm
   before deploying to the ADuC841.
   
   Compile: gcc rx_verification_test.c -o rx_test -Wall -Wextra -std=c99
   Run:     ./rx_test
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ========================================================================== */
/* RX Syndrome Computation (from rx_decoder.c) */
/* ========================================================================== */

#define BUS_STATE_MASK 0x7FFF
#define HAMMING_N 15

uint8_t rx_compute_syndrome(uint16_t bus_state)
{
    uint8_t syndrome = 0;
    uint8_t col_idx;
    uint16_t temp_state;
    
    temp_state = bus_state & BUS_STATE_MASK;
    
    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp_state & 0x0001)
        {
            syndrome ^= col_idx;
        }
        temp_state >>= 1;
    }
    
    return (syndrome & 0x0F);
}

/* ========================================================================== */
/* TX Syndrome Computation (from bus_encoder.c) - for comparison */
/* ========================================================================== */

uint8_t tx_compute_syndrome(uint16_t bus_state)
{
    uint8_t syndrome = 0;
    uint8_t col_idx;
    uint16_t temp_state;
    
    temp_state = bus_state & BUS_STATE_MASK;
    
    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp_state & 0x0001)
        {
            syndrome ^= col_idx;
        }
        temp_state >>= 1;
    }
    
    return syndrome;
}

/* ========================================================================== */
/* Test Vectors */
/* ========================================================================== */

typedef struct {
    uint16_t bus_state;
    uint8_t expected_syndrome;
    const char* description;
} test_vector_t;

test_vector_t test_vectors[] = {
    /* Single bit tests (each column) */
    {0x0000, 0x0, "All zeros"},
    {0x0001, 0x1, "Bit 0 (column 1)"},
    {0x0002, 0x2, "Bit 1 (column 2)"},
    {0x0004, 0x3, "Bit 2 (column 3)"},
    {0x0008, 0x4, "Bit 3 (column 4)"},
    {0x0010, 0x5, "Bit 4 (column 5)"},
    {0x0020, 0x6, "Bit 5 (column 6)"},
    {0x0040, 0x7, "Bit 6 (column 7)"},
    {0x0080, 0x8, "Bit 7 (column 8)"},
    {0x0100, 0x9, "Bit 8 (column 9)"},
    {0x0200, 0xA, "Bit 9 (column 10)"},
    {0x0400, 0xB, "Bit 10 (column 11)"},
    {0x0800, 0xC, "Bit 11 (column 12)"},
    {0x1000, 0xD, "Bit 12 (column 13)"},
    {0x2000, 0xE, "Bit 13 (column 14)"},
    {0x4000, 0xF, "Bit 14 (column 15)"},
    
    /* Multi-bit tests */
    {0x0003, 0x3, "Bits 0,1: 1^2=3"},
    {0x000F, 0x4, "Bits 0-3: 1^2^3^4=4"},
    {0x00FF, 0x8, "Bits 0-7: XOR=8"},
    {0x7FFF, 0x0, "All bits: XOR=0"},
    
    /* ASCII encoding examples */
    /* 'A' = 0x41: high nibble=4, low nibble=1 */
    {0x0008, 0x4, "Encodes nibble 4 (high of 'A')"},
    {0x0001, 0x1, "Encodes nibble 1 (low of 'A')"},
    
    /* 'B' = 0x42: high nibble=4, low nibble=2 */
    {0x0008, 0x4, "Encodes nibble 4 (high of 'B')"},
    {0x0002, 0x2, "Encodes nibble 2 (low of 'B')"},
};

/* ========================================================================== */
/* Test Runner */
/* ========================================================================== */

int main(void)
{
    int num_tests = sizeof(test_vectors) / sizeof(test_vectors[0]);
    int passed = 0;
    int failed = 0;
    int i;
    
    printf("═══════════════════════════════════════════════════════════\n");
    printf("RX Syndrome Computation Verification Test\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    
    /* Test 1: Verify RX matches expected values */
    printf("TEST 1: RX Syndrome Computation\n");
    printf("───────────────────────────────────────────────────────────\n");
    
    for (i = 0; i < num_tests; i++)
    {
        test_vector_t* tv = &test_vectors[i];
        uint8_t result = rx_compute_syndrome(tv->bus_state);
        
        printf("Test %2d: %s\n", i+1, tv->description);
        printf("  Bus State: 0x%04X\n", tv->bus_state);
        printf("  Expected:  0x%X\n", tv->expected_syndrome);
        printf("  Result:    0x%X\n", result);
        
        if (result == tv->expected_syndrome) {
            printf("  Status:    ✓ PASS\n");
            passed++;
        } else {
            printf("  Status:    ✗ FAIL\n");
            failed++;
        }
        printf("\n");
    }
    
    printf("───────────────────────────────────────────────────────────\n");
    printf("Test 1 Summary: %d/%d passed\n\n", passed, num_tests);
    
    /* Test 2: Verify RX matches TX exactly */
    printf("TEST 2: RX vs TX Syndrome Comparison\n");
    printf("───────────────────────────────────────────────────────────\n");
    
    int mismatch_count = 0;
    for (uint16_t state = 0; state <= 0x7FFF; state++)
    {
        uint8_t rx_result = rx_compute_syndrome(state);
        uint8_t tx_result = tx_compute_syndrome(state);
        
        if (rx_result != tx_result)
        {
            printf("MISMATCH at 0x%04X: RX=0x%X, TX=0x%X\n",
                   state, rx_result, tx_result);
            mismatch_count++;
            
            if (mismatch_count >= 10) {
                printf("(stopping after 10 mismatches)\n");
                break;
            }
        }
        
        /* Progress indicator */
        if (state % 0x1000 == 0) {
            printf("  Tested %5d / 32768 states...\r", state);
            fflush(stdout);
        }
    }
    
    if (mismatch_count == 0) {
        printf("✓ PASS: RX and TX produce IDENTICAL results for all 32768 states\n");
    } else {
        printf("✗ FAIL: Found %d mismatches\n", mismatch_count);
    }
    printf("\n");
    
    /* Test 3: ASCII Character Encoding/Decoding */
    printf("TEST 3: ASCII Character Round-Trip\n");
    printf("───────────────────────────────────────────────────────────\n");
    
    const char* test_string = "HELLO WORLD!";
    int string_len = strlen(test_string);
    int char_errors = 0;
    
    printf("Test String: \"%s\"\n\n", test_string);
    
    for (i = 0; i < string_len; i++)
    {
        uint8_t original_char = (uint8_t)test_string[i];
        uint8_t high_nibble = (original_char >> 4) & 0x0F;
        uint8_t low_nibble = original_char & 0x0F;
        
        /* Simulate TX encoding (simplified: use single-bit solution) */
        uint16_t bus_state_high = (high_nibble == 0) ? 0 : (1 << (high_nibble - 1));
        uint16_t bus_state_low = (low_nibble == 0) ? 0 : (1 << (low_nibble - 1));
        
        /* RX decoding */
        uint8_t decoded_high = rx_compute_syndrome(bus_state_high);
        uint8_t decoded_low = rx_compute_syndrome(bus_state_low);
        uint8_t reconstructed_char = (decoded_high << 4) | decoded_low;
        
        printf("Char %2d: '%c' (0x%02X)\n", i+1, original_char, original_char);
        printf("  High nibble: 0x%X → bus 0x%04X → decoded 0x%X\n",
               high_nibble, bus_state_high, decoded_high);
        printf("  Low nibble:  0x%X → bus 0x%04X → decoded 0x%X\n",
               low_nibble, bus_state_low, decoded_low);
        printf("  Reconstructed: '%c' (0x%02X)\n",
               reconstructed_char, reconstructed_char);
        
        if (reconstructed_char == original_char) {
            printf("  Status: ✓ PASS\n");
        } else {
            printf("  Status: ✗ FAIL\n");
            char_errors++;
        }
        printf("\n");
    }
    
    printf("───────────────────────────────────────────────────────────\n");
    printf("Test 3 Summary: %d/%d characters decoded correctly\n\n",
           string_len - char_errors, string_len);
    
    /* Final Summary */
    printf("═══════════════════════════════════════════════════════════\n");
    printf("FINAL SUMMARY\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Test 1 (RX Syndrome):     %d/%d passed\n", passed, num_tests);
    printf("Test 2 (RX vs TX):        %s\n",
           mismatch_count == 0 ? "PASS" : "FAIL");
    printf("Test 3 (ASCII Round-Trip): %d/%d passed\n",
           string_len - char_errors, string_len);
    printf("\n");
    
    if (failed == 0 && mismatch_count == 0 && char_errors == 0) {
        printf("═══════════════════════════════════════════════════════════\n");
        printf("✓ ALL TESTS PASSED\n");
        printf("RX decoder is verified and ready for deployment!\n");
        printf("═══════════════════════════════════════════════════════════\n");
        return 0;
    } else {
        printf("═══════════════════════════════════════════════════════════\n");
        printf("✗ SOME TESTS FAILED\n");
        printf("Review errors above before deploying to ADuC841\n");
        printf("═══════════════════════════════════════════════════════════\n");
        return 1;
    }
}

/* ========================================================================== */
/* Additional Utility Functions */
/* ========================================================================== */

/* Print binary representation of 15-bit value */
void print_binary_15bit(uint16_t value)
{
    int i;
    printf("0b");
    for (i = 14; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
        if (i % 4 == 0 && i != 0) printf(" ");
    }
}

/* Generate H1 matrix column i */
uint8_t h1_column(uint8_t i)
{
    return i & 0x0F;  /* Column i = binary value of i */
}

/* Print H1 matrix (for reference) */
void print_h1_matrix(void)
{
    int col, row;
    
    printf("\nH1 Matrix (4×15):\n");
    printf("     ");
    for (col = 1; col <= 15; col++) {
        printf("%2d ", col);
    }
    printf("\n");
    
    for (row = 0; row < 4; row++) {
        printf("Row %d: ", row);
        for (col = 1; col <= 15; col++) {
            printf(" %d ", (col >> row) & 1);
        }
        printf("\n");
    }
    printf("\n");
}
