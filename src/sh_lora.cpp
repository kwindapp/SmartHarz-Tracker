/**
 * @file sh_lora.cpp
 * @brief LilyGO T-Echo SX1262 Helium LoRaWAN implementation
 */

#include "sh_lora.h"

// T-Echo SX1262 pin order:
// NSS, DIO1, RESET, BUSY
SX1262 radio = new Module(
    LORA_CS,
    LORA_DIO1,
    LORA_RST,
    LORA_BUSY
);

// EU868 LoRaWAN region
LoRaWANNode node(&radio, &EU868);

// Helium OTAA credentials.
// RadioLib uses the normal EUI order.
static const uint64_t DEV_EUI =
    0x6081F9FD413E986AULL;

static const uint64_t JOIN_EUI =
    0x6081F9A18AEB66CBULL;

static const uint8_t APP_KEY[16] = {
    0xBE, 0xAE, 0x71, 0xDE,
    0xC5, 0x43, 0x9E, 0x1A,
    0x52, 0x1C, 0x12, 0xC1,
    0xC3, 0x1A, 0x06, 0xDB
};

bool loraInit()
{
    SERIAL_MON.println();
    SERIAL_MON.println("[LoRaWAN] Starting...");

    // Configure the nRF52840 SPI pins for the T-Echo radio.
    SPI.setPins(
        LORA_MISO,
        LORA_SCLK,
        LORA_MOSI
    );

    SPI.begin();

    SERIAL_MON.println("[LoRaWAN] Initializing SX1262...");

    int16_t state = radio.begin(868.0);

    if (state != RADIOLIB_ERR_NONE)
    {
        SERIAL_MON.print(
            "[LoRaWAN] SX1262 initialization failed: "
        );
        SERIAL_MON.println(state);
        return false;
    }

    SERIAL_MON.println("[LoRaWAN] SX1262 initialized");

    // Helium device is configured for LoRaWAN 1.0.x.
    // Therefore NwkKey is NULL and AppKey is supplied.
    state = node.beginOTAA(
        JOIN_EUI,
        DEV_EUI,
        NULL,
        APP_KEY
    );

    if (state != RADIOLIB_ERR_NONE)
    {
        SERIAL_MON.print(
            "[LoRaWAN] OTAA configuration failed: "
        );
        SERIAL_MON.println(state);
        return false;
    }

    SERIAL_MON.println(
        "[LoRaWAN] Sending Helium join request..."
    );

    state = node.activateOTAA();

    if (state == RADIOLIB_LORAWAN_NEW_SESSION)
    {
        SERIAL_MON.println(
            "[LoRaWAN] Joined Helium successfully"
        );
        return true;
    }

    if (state == RADIOLIB_LORAWAN_SESSION_RESTORED)
    {
        SERIAL_MON.println(
            "[LoRaWAN] Previous session restored"
        );
        return true;
    }

    SERIAL_MON.print("[LoRaWAN] Join failed: ");
    SERIAL_MON.println(state);

    return false;
}

bool loraSendTest()
{
    static const uint8_t payload[] = {
        'K', 'W', 'i', 'n', 'd'
    };

    SERIAL_MON.println("[LoRaWAN] Sending test uplink...");

    int16_t state = node.sendReceive(
        payload,
        sizeof(payload),
        1,      // FPort
        false   // Unconfirmed uplink
    );

    if (state < RADIOLIB_ERR_NONE)
    {
        SERIAL_MON.print("[LoRaWAN] Uplink failed: ");
        SERIAL_MON.println(state);
        return false;
    }

    SERIAL_MON.println("[LoRaWAN] Uplink sent");

    if (state > 0)
    {
        SERIAL_MON.print(
            "[LoRaWAN] Downlink received in RX window "
        );
        SERIAL_MON.println(state);
    }
    else
    {
        SERIAL_MON.println("[LoRaWAN] No downlink");
    }

    return true;
}

void loraLoop()
{
    // No continuous processing is required for Class A.
}