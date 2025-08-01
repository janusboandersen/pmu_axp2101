#pragma once

// Convenience for masking e.g. uint8_t types
#define _BIT(N) (1U << (N))   // Mask Nth bit 
#define MASK_BITS_4_0 0x1F    // Mask bits 4:0 (5 bits)
#define MASK_BITS_6_0 0x7F    // Mask bits 6:0 (7 bits)
#define SENTINEL_8BIT 0xFF    // Inline function will return on error
