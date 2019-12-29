#include <stdint.h>

#define LCW_TANH_POW2_FRACTION_BITS (8)
#define LCW_TANH_POW2_TABLE_SIZE (1025)

#define LCW_TANH_POW2_VALUE_BITS (14)
#define LCW_TANH_POW2_VALUE_MAX (1 << (LCW_TANH_POW2_VALUE_BITS))

extern const int16_t gLcwTanhPow2Table[LCW_TANH_POW2_TABLE_SIZE];

