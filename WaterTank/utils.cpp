#include "utils.h"

uint8_t int32ToStrFixedPoint(int32_t value, char buffer[13], uint8_t point_position) {
    int len = 0;
    if(value < 0) {
        value = -value;
        buffer[0] = '-';
        len++;
    }

    const int32_t exponent[10] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

    bool number_start = false;
    
    for (int exp = 0; exp < 10; exp++) {
        buffer[len] = '0';
        while (value >= exponent[exp]) {
            value -= exponent[exp];
            buffer[len]++;
        }

        bool point = (exp == (10 - 1 - point_position));

        if((buffer[len] != '0') || number_start || point)  {
            number_start = true;
            len++;
        }

        if(point && (point_position != 0)) {
            buffer[len] = '.';
            len++;
        }
    }
    buffer[len] = 0;
    return len;
}
