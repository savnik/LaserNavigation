#include "crc.h"

#define WIDTH    (8 * sizeof(crc))
#define TOPBIT   (1 << (WIDTH - 1))

crc getCRCint(signed long int const msg,int nBytes)
{
    crc            remainder = INITIAL_REMAINDER;
        int            byte;
        unsigned char  bit;
    unsigned char *msgbytes  = (unsigned char*)&msg;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= *msgbytes++ << (WIDTH - 8); //do next byte  //update CRC
        for (bit = 8; bit > 0; --bit)
        {
            if (remainder & TOPBIT)  //if LSB XOR == 1
                remainder = (remainder << 1) ^ POLYNOMIAL;  //then XOR polynomial with CRC
            else
                remainder = (remainder << 1);
        }
    }
    return remainder ^ FINAL_XOR_VALUE;
}


crc getCRC(unsigned char const message[], int nBytes)
{
    crc            remainder = INITIAL_REMAINDER;
	int            byte;
	unsigned char  bit;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte] << (WIDTH - 8); //do next bit  //update CRC
        for (bit = 8; bit > 0; --bit)
        {
            if (remainder & TOPBIT)  //if LSB XOR == 1
                remainder = (remainder << 1) ^ POLYNOMIAL;  //then XOR polynomial with CRC
            else
                remainder = (remainder << 1);
        }
    }
    return remainder ^ FINAL_XOR_VALUE;
} 





