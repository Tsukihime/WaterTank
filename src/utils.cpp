#include "utils.h"

#ifdef __AVR__
    #include <avr/pgmspace.h>
#else
    #define PROGMEM
    #define pgm_read_dword_near(addr) (*(addr))
#endif

uint8_t int32ToStrFixedPoint(int32_t value, char buffer[13], uint8_t point_position) {
    uint8_t len = 0;
    uint32_t u_value;

    if(value < 0) {
        u_value = -static_cast<uint32_t>(value);
        buffer[len++] = '-';
    } else {
        u_value = value;
    }

    static const uint32_t exp[10] PROGMEM = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

    bool print_start = false;

    for (int8_t i = 9; i >= 0; i--) {
        char digit = '0';
        uint32_t exponent = pgm_read_dword_near(&exp[i]);
        while (u_value >= exponent) {
            u_value -= exponent;
            digit++;
        }

        bool point_expected = (i == point_position);
        if (!print_start && digit == '0' && !point_expected) {
            continue; // Skip leading zeros
        }

        print_start = true;
        buffer[len++] = digit;

        if(point_expected && (point_position != 0)) {
            buffer[len++] = '.';
        }
    }
    buffer[len] = '\0';
    return len;
}
