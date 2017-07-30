#include "fletcher.h"

uint32_t fletcher32( void* d, int words )
{
        uint16_t* data = (uint16_t*)d;
        uint32_t sum1 = 0xffff, sum2 = 0xffff;
        int tlen;
 
        while (words) {
                tlen = ((words >= 359) ? 359 : words);
                words -= tlen;
                do {
                        sum2 += sum1 += *data++;
                        tlen--;
                } while (tlen);
                sum1 = (sum1 & 0xffff) + (sum1 >> 16);
                sum2 = (sum2 & 0xffff) + (sum2 >> 16);
        }
        /* Second reduction step to reduce sums to 16 bits */
        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
        return (sum2 << 16) | sum1;
}