/**
 * @file credentials_template.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * This is nothing but an empty file for the GitHub Actions to run
 * 
 */

#ifndef _CREDENTIALS_H_
#define _CREDENTIALS_H_


#include <lmic.h>
#include <avr/pgmspace.h>

/**
 * This EUI must be in little-endian format, so least-significant-byte
 * first. When copying an EUI from ttnctl output, this means to reverse
 * the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
 * 0x70.
 */
static const u1_t PROGMEM DEVEUI[8]={ 0x60, 0x81, 0xF9, 0x31, 0x39, 0x5E, 0x81, 0x2B };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

/** This should also be in little endian format, see above. */
static const u1_t PROGMEM APPEUI[8]={  0x60, 0x81, 0xF9, 0xA1, 0x8A, 0xEB, 0x66, 0xCB };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

/**
 * This key should be in big endian format (or, since it is not really a
 * number but a block of memory, endianness does not really apply). In
 * practice, a key taken from ttnctl can be copied as-is.
 */
static const u1_t PROGMEM APPKEY[16] = { 0x5F, 0xC9, 0x8E, 0xD2, 0x05, 0x70, 0xD0, 0x87, 0xFF, 0xBB, 0xE3, 0x37, 0x5E, 0x12, 0x0C, 0xC3};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

#endif // _CREDENTIALS_H_