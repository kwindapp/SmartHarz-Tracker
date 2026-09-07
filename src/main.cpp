/**
 * @file main.cpp
 * @brief LilyGO T-Echo Helium LoRaWAN connection test
 */

#include "main.h"
#include "sh_lora.h"

static bool loraConnected = false;
static uint32_t lastSendTime = 0;

// Send a test message every 60 seconds.
static const uint32_t SEND_INTERVAL_MS = 60000UL;

void setup()
{
    SERIAL_MON.begin(115200);
    delay(2000);

    SERIAL_MON.println();
    SERIAL_MON.println("================================");
    SERIAL_MON.println("LilyGO T-Echo");
    SERIAL_MON.println("Helium LoRaWAN OTAA");
    SERIAL_MON.println("Region: EU868");
    SERIAL_MON.println("================================");

    // Initialize the T-Echo board and power control.
    setupBoard();

    delay(1000);

    SERIAL_MON.println("Connecting to Helium...");

    loraConnected = loraInit();

    if (loraConnected)
    {
        SERIAL_MON.println();
        SERIAL_MON.println("LoRaWAN connection successful");
        SERIAL_MON.println("Sending first test uplink...");

        if (loraSendTest())
        {
            SERIAL_MON.println("First uplink completed");
        }
        else
        {
            SERIAL_MON.println("First uplink failed");
        }

        lastSendTime = millis();
    }
    else
    {
        SERIAL_MON.println();
        SERIAL_MON.println("LoRaWAN connection failed");
        SERIAL_MON.println("Check antenna, coverage and OTAA keys");
    }
}

void loop()
{
    loraLoop();

    if (loraConnected &&
        millis() - lastSendTime >= SEND_INTERVAL_MS)
    {
        lastSendTime = millis();

        SERIAL_MON.println();
        SERIAL_MON.println("Sending scheduled test uplink...");

        loraSendTest();
    }

    delay(10);
}