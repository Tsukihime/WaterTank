#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

namespace bit {
    template <typename T1, typename T2> inline void set  (T1 &variable, T2 bit) {variable |=  ((T1)1 << bit);}
    template <typename T1, typename T2> inline void clear(T1 &variable, T2 bit) {variable &= ~((T1)1 << bit);}
    template <typename T1, typename T2> inline void flip (T1 &variable, T2 bit) {variable ^=  ((T1)1 << bit);}
    template <typename T1, typename T2> inline bool test (T1 &variable, T2 bit) {return variable & ((T1)1 << bit);}
}

namespace bitmask {
    template <typename T1, typename T2> inline void set  (T1 &variable, T2 bits) {variable |= bits;}
    template <typename T1, typename T2> inline void clear(T1 &variable, T2 bits) {variable &= ~bits;}
    template <typename T1, typename T2> inline void flip (T1 &variable, T2 bits) {variable ^= bits;}
    template <typename T1, typename T2> inline bool test_all(T1 &variable, T2 bits) {return ((variable & bits) == bits);}
    template <typename T1, typename T2> inline bool test_any(T1 &variable, T2 bits) {return variable & bits;}
}

template <typename T1, typename T2> inline T1 round_div(T1 numerator, T2 denominator) {return ((numerator + (denominator >> 1)) / denominator);}
template <typename T1, typename T2> constexpr T1 c_round_div(T1 numerator, T2 denominator) {return ((numerator + (denominator >> 1)) / denominator);}

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi ) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

inline void bin2bcd10(uint32_t value, uint8_t buffer[10]) {
    uint32_t subtrahend = 1000000000;
    for (uint8_t i = 0; i < 10; i++) {
        buffer[i] = 0;
        while (value >= subtrahend) {
            value -= subtrahend;
            buffer[i]++;
        }
        subtrahend /= 10;
    }
}

inline void bin2bcd5(uint16_t value, uint8_t buffer[5]) {
    buffer[0] = 0;
    while (value > 9999) {
        value -= 10000;
        buffer[0]++;
    }

    buffer[1] = 0;
    while (value > 999) {
        value -= 1000;
        buffer[1]++;
    }

    buffer[2] = 0;
    while (value > 99) {
        value -= 100;
        buffer[2]++;
    }

    buffer[3] = 0;
    while (value > 9) {
        value -= 10;
        buffer[3]++;
    }

    buffer[4] = value;
}

inline void bcd2ascii(uint8_t ints[5]) {
    uint8_t i;
    for(i = 0; i < 5; i++) {
        ints[i] += '0';
    }
}

uint8_t int32ToStrFixedPoint(int32_t value, char buffer[13], uint8_t point_position = 0);

#endif /* UTILS_H_ */
