/**
 * @file credentials.h
 * @brief Helium LoRaWAN OTAA credentials for LilyGO T-Echo
 */

#ifndef _CREDENTIALS_H_
#define _CREDENTIALS_H_

#include <lmic.h>
#include <avr/pgmspace.h>

// DevEUI shown in Helium:
// 60 81 F9 FD 41 3E 98 6A
//
// LMIC requires little-endian byte order:
static const u1_t PROGMEM DEVEUI[8] = {
    0x6A, 0x98, 0x3E, 0x41,
    0xFD, 0xF9, 0x81, 0x60
};

void os_getDevEui(u1_t* buf)
{
    memcpy_P(buf, DEVEUI, sizeof(DEVEUI));
}

// JoinEUI/AppEUI shown in Helium:
// 60 81 F9 A1 8A EB 66 CB
//
// LMIC requires little-endian byte order:
static const u1_t PROGMEM APPEUI[8] = {
    0xCB, 0x66, 0xEB, 0x8A,
    0xA1, 0xF9, 0x81, 0x60
};

void os_getArtEui(u1_t* buf)
{
    memcpy_P(buf, APPEUI, sizeof(APPEUI));
}

// AppKey stays in the same order shown in Helium.
static const u1_t PROGMEM APPKEY[16] = {
    0xBE, 0xAE, 0x71, 0xDE,
    0xC5, 0x43, 0x9E, 0x1A,
    0x52, 0x1C, 0x12, 0xC1,
    0xC3, 0x1A, 0x06, 0xDB
};

void os_getDevKey(u1_t* buf)
{
    memcpy_P(buf, APPKEY, sizeof(APPKEY));
}

#endif // _CREDENTIALS_H_