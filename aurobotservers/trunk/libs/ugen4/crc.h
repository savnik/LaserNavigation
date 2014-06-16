/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file describing the various CRC standards.
 *
 * Notes:       
 *
 * 
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef _crc_h
#define _crc_h

#ifdef __cplusplus
extern "C" {
#else
//  #define FALSE 0
//  #define TRUE  !FALSE
#endif

#define CRC8

#if defined(CRC_CCITT)
typedef unsigned short  crc;
#define CRC_NAME			"CRC-CCITT"
#define POLYNOMIAL			0x1021
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define CRCLEN 2

#elif defined(CRC8)
typedef unsigned char  crc; 
#define CRC_NAME			"CRC8-CCITT"
#define POLYNOMIAL			0x07 
#define INITIAL_REMAINDER	0xFF
#define FINAL_XOR_VALUE		0x00
#define CRCLEN 1

#elif defined(CRC16)

typedef unsigned short  crc;

#define CRC_NAME			"CRC-16"
#define POLYNOMIAL			0x8005
#define INITIAL_REMAINDER	0x0000
#define FINAL_XOR_VALUE		0x0000
#define CRCLEN 2
#elif defined(CRC32)

typedef unsigned long  crc;

#define CRC_NAME			"CRC-32"
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define CRCLEN 4
#else

#error "One of CRC_CCITT, CRC8, CRC16, or CRC32 must be #define'd."

#endif


crc   getCRC(unsigned char const message[], int nBytes);
/**
 * calculate the remaining CRC value from n bytes long integer.
 * if the value has no CRC, then the result may be added as least significant byte.
 * \param msg is the bytes to check
 * \param nBytes is the number of bytes to use.
 * \returns 0 if crc check is correct
 * \returns 8-bit crc value to append to get 0 in a follow-on check. */ 
crc   getCRCint(signed long int const msg,int nBytes);

#ifdef __cplusplus
}
#endif

#endif /* _crc_h */
