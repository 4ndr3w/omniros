#include "fletcher.h"

uint16_t fletcher16( void *d, int bytes )
{
        uint8_t* data = (uint8_t*)d;
        uint16_t sum1 = 0xff, sum2 = 0xff;
        int tlen;
 
        while (bytes) {
                tlen = ((bytes >= 20) ? 20 : bytes);
                bytes -= tlen;
                do {
                        sum2 += sum1 += *data++;
                        tlen--;
                } while (tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
        return (sum2 << 8) | sum1;
}

