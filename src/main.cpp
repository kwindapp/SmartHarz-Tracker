/**
 * @file main.cpp
 *
 * KWind WS80 + LilyGO T-Echo + GPS
 * Helium LoRaWAN EU868
 *
 * Hardware:
 * - LilyGO T-Echo
 * - nRF52840
 * - SX1262
 * - WS80 ultrasonic wind station
 *
 * WS80 wiring:
 * - WS80 TX  -> T-Echo SDA / GPIO26
 * - WS80 GND -> T-Echo GND
 *
 * Important:
 * - WS80 is receive-only.
 * - Do not call setupBoard(), because it starts I2C on GPIO26/27.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <math.h>

#include "t-echo.h"

// =============================================================================
// LORAWAN OTAA CREDENTIALS
// =============================================================================

// RadioLib uses the normal hexadecimal EUI order.
static const uint64_t DEV_EUI =
    0x6081F9FD413E986AULL;

static const uint64_t JOIN_EUI =
    0x6081F9A18AEB66CBULL;

static const uint8_t APP_KEY[16] = {
   0xBA, 0x86, 0x05, 0x11, 0x6F, 0x4D, 0x7F, 0x7F, 0xCA, 0xB3, 0x9C, 0x49, 0x99, 0xD8, 0x28, 0xC1
};

// =============================================================================
// SETTINGS
// =============================================================================

static constexpr uint8_t UPLINK_PORT = 2;

static constexpr uint32_t DEFAULT_SEND_INTERVAL_MS =
   120000UL;

static uint32_t sendIntervalMs =
    DEFAULT_SEND_INTERVAL_MS;

static constexpr uint32_t SENSOR_STALE_MS =
    30000UL;

static constexpr uint32_t GPS_STALE_MS =
    300000UL;

static constexpr uint32_t JOIN_RETRY_MS =
    60000UL;

static constexpr uint32_t STATUS_INTERVAL_MS =
    10000UL;

static constexpr uint32_t WS80_BAUD =
    115200UL;

static constexpr uint32_t GPS_BAUD =
    9600UL;

// =============================================================================
// RADIO OBJECTS
// =============================================================================

SPIClass* radioSpi = nullptr;
SX1262* radio = nullptr;
LoRaWANNode* lorawan = nullptr;

// T-Echo 1.54-inch 200 x 200 e-paper display.
SPIClass* displaySpi = nullptr;

GxEPD2_BW<
    GxEPD2_154_D67,
    GxEPD2_154_D67::HEIGHT
> display(
    GxEPD2_154_D67(
        EPD_CS,
        EPD_DC,
        EPD_RST,
        EPD_BUSY
    )
);

static bool displayReady = false;
static uint8_t lastDownlinkPort = 0;
static size_t lastDownlinkLength = 0;

// =============================================================================
// GPS
// =============================================================================

TinyGPSPlus gps;

struct GpsFix
{
    double latitude = 0.0;
    double longitude = 0.0;

    uint8_t satellites = 0;
    uint8_t hdopX10 = 0xFF;

    uint32_t updatedMs = 0;

    bool valid = false;
};

GpsFix gpsFix;

// =============================================================================
// WS80 DATA
// =============================================================================

struct Ws80Measurement
{
    float wind = NAN;
    float gust = NAN;
    float temperature = NAN;
    float battery = NAN;
    float uv = NAN;

    uint16_t direction = 0;
    uint8_t humidity = 0xFF;

    uint32_t light = 0xFFFFFFFFUL;
    uint32_t updatedMs = 0;

    bool valid = false;
};

Ws80Measurement ws80;

// =============================================================================
// WIND STATISTICS
// =============================================================================

struct WindStatistics
{
    double windSum = 0.0;
    double directionSinSum = 0.0;
    double directionCosSum = 0.0;

    float maximumGust = 0.0f;
    float minimumWind = 0.0f;

    uint32_t count = 0;

    void reset()
    {
        windSum = 0.0;
        directionSinSum = 0.0;
        directionCosSum = 0.0;

        maximumGust = 0.0f;
        minimumWind = 0.0f;

        count = 0;
    }

    void add(
        float wind,
        float gust,
        uint16_t direction
    )
    {
        if (!isfinite(wind) ||
            !isfinite(gust) ||
            wind < 0.0f ||
            gust < 0.0f ||
            direction >= 360)
        {
            return;
        }

        windSum += wind;

        if (count == 0)
        {
            maximumGust = gust;
            minimumWind = wind;
        }
        else
        {
            if (gust > maximumGust)
            {
                maximumGust = gust;
            }

            if (wind < minimumWind)
            {
                minimumWind = wind;
            }
        }

        // Weight wind direction by wind speed.
        if (wind > 0.05f)
        {
            const double radians =
                direction * PI / 180.0;

            directionSinSum +=
                sin(radians) * wind;

            directionCosSum +=
                cos(radians) * wind;
        }

        count++;
    }

    float averageWind() const
    {
        if (count == 0)
        {
            return NAN;
        }

        return static_cast<float>(
            windSum / count
        );
    }

    float averageDirection(
        uint16_t fallback
    ) const
    {
        if (directionSinSum == 0.0 &&
            directionCosSum == 0.0)
        {
            return fallback;
        }

        double direction =
            atan2(
                directionSinSum,
                directionCosSum
            ) * 180.0 / PI;

        if (direction < 0.0)
        {
            direction += 360.0;
        }

        return static_cast<float>(direction);
    }
};

WindStatistics windStats;

// =============================================================================
// STATE
// =============================================================================

static char ws80Line[128];
static size_t ws80LineLength = 0;

static bool haveDirection = false;
static bool haveWind = false;
static bool haveGust = false;
static bool readingStored = false;

static bool lorawanJoined = false;

static uint32_t lastSendMs = 0;
static uint32_t lastJoinAttemptMs = 0;
static uint32_t lastStatusMs = 0;

// =============================================================================
// HELPERS
// =============================================================================

static const char* valueAfterEquals(
    const char* text
)
{
    const char* value = strchr(text, '=');

    if (value == nullptr)
    {
        return nullptr;
    }

    value++;

    while (*value == ' ' || *value == '\t')
    {
        value++;
    }

    return value;
}

static uint16_t scaleUnsigned16(
    float value,
    float multiplier
)
{
    if (!isfinite(value) || value < 0.0f)
    {
        return 0;
    }

    const float scaled =
        value * multiplier;

    if (scaled >= 65535.0f)
    {
        return 65535;
    }

    return static_cast<uint16_t>(
        scaled + 0.5f
    );
}

static void putUint16(
    uint8_t* buffer,
    size_t& position,
    uint16_t value
)
{
    buffer[position++] =
        static_cast<uint8_t>(value >> 8);

    buffer[position++] =
        static_cast<uint8_t>(value);
}

static void putUint32(
    uint8_t* buffer,
    size_t& position,
    uint32_t value
)
{
    buffer[position++] =
        static_cast<uint8_t>(value >> 24);

    buffer[position++] =
        static_cast<uint8_t>(value >> 16);

    buffer[position++] =
        static_cast<uint8_t>(value >> 8);

    buffer[position++] =
        static_cast<uint8_t>(value);
}

// =============================================================================
// T-ECHO BATTERY
// =============================================================================

static uint16_t readBoardBatteryMv()
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        sum += analogRead(ADC_PIN);
        delayMicroseconds(200);
    }

    const float average =
        sum / 8.0f;

    // T-Echo battery ADC uses an approximately 2:1 divider.
    const float millivolts =
        average *
        (3000.0f / 4095.0f) *
        2.0f;

    return static_cast<uint16_t>(
        constrain(
            static_cast<int>(millivolts + 0.5f),
            0,
            65534
        )
    );
}

// =============================================================================
// WS80 PARSER
// =============================================================================

static void storeCompleteWindReading()
{
    if (readingStored)
    {
        return;
    }

    if (!haveDirection ||
        !haveWind ||
        !haveGust)
    {
        return;
    }

    windStats.add(
        ws80.wind,
        ws80.gust,
        ws80.direction
    );

    readingStored = true;
}

static void parseWs80Line(
    const char* line
)
{
    const char* value =
        valueAfterEquals(line);

    if (value == nullptr)
    {
        return;
    }

    if (strncmp(line, "WindDir", 7) == 0)
    {
        const long parsed =
            strtol(value, nullptr, 10);

        if (parsed >= 0 && parsed <= 360)
        {
            ws80.direction =
                static_cast<uint16_t>(
                    parsed % 360
                );

            // Beginning of a new WS80 data block.
            ws80.valid = false;

            ws80.temperature = NAN;
            ws80.battery = NAN;
            ws80.uv = NAN;

            ws80.humidity = 0xFF;
            ws80.light = 0xFFFFFFFFUL;

            haveDirection = true;
            haveWind = false;
            haveGust = false;
            readingStored = false;
        }
    }
    else if (
        strncmp(line, "WindSpeed", 9) == 0
    )
    {
        const float parsed =
            strtof(value, nullptr);

        if (isfinite(parsed) &&
            parsed >= 0.0f)
        {
            ws80.wind = parsed;
            haveWind = true;

            storeCompleteWindReading();
        }
    }
    else if (
        strncmp(line, "WindGust", 8) == 0
    )
    {
        const float parsed =
            strtof(value, nullptr);

        if (isfinite(parsed) &&
            parsed >= 0.0f)
        {
            ws80.gust = parsed;
            haveGust = true;

            storeCompleteWindReading();
        }
    }
    else if (
        strncmp(line, "Temperature", 11) == 0
    )
    {
        const float parsed =
            strtof(value, nullptr);

        if (isfinite(parsed) &&
            parsed >= -100.0f &&
            parsed <= 100.0f)
        {
            ws80.temperature = parsed;
        }
    }
    else if (
        strncmp(line, "Humi", 4) == 0
    )
    {
        const long parsed =
            strtol(value, nullptr, 10);

        if (parsed >= 0 && parsed <= 100)
        {
            ws80.humidity =
                static_cast<uint8_t>(parsed);
        }
    }
    else if (
        strncmp(line, "Light", 5) == 0
    )
    {
        const long parsed =
            strtol(value, nullptr, 10);

        if (parsed >= 0)
        {
            ws80.light =
                static_cast<uint32_t>(parsed);
        }
    }
    else if (
        strncmp(line, "UV_Value", 8) == 0
    )
    {
        const float parsed =
            strtof(value, nullptr);

        if (isfinite(parsed) &&
            parsed >= 0.0f)
        {
            ws80.uv = parsed;
        }
    }
    else if (
        strncmp(line, "BatVoltage", 10) == 0
    )
    {
        const float parsed =
            strtof(value, nullptr);

        if (isfinite(parsed) &&
            parsed >= 0.0f &&
            parsed <= 10.0f &&
            haveDirection &&
            haveWind &&
            haveGust)
        {
            ws80.battery = parsed;
            ws80.updatedMs = millis();
            ws80.valid = true;

            storeCompleteWindReading();
        }
    }
}

static void pollWs80()
{
    while (Serial1.available() > 0)
    {
        const char character =
            static_cast<char>(Serial1.read());

        if (character == '\r' ||
            character == '\n')
        {
            if (ws80LineLength > 0)
            {
                ws80Line[ws80LineLength] = '\0';

                parseWs80Line(ws80Line);

                ws80LineLength = 0;
            }
        }
        else if (
            character >= 32 &&
            character <= 126
        )
        {
            if (
                ws80LineLength <
                sizeof(ws80Line) - 1
            )
            {
                ws80Line[ws80LineLength++] =
                    character;
            }
            else
            {
                // Discard an overlong or corrupt line.
                ws80LineLength = 0;
            }
        }
    }

    if (ws80.valid &&
        millis() - ws80.updatedMs >
            SENSOR_STALE_MS)
    {
        ws80.valid = false;
    }
}

// =============================================================================
// GPS
// =============================================================================

static void pollGps()
{
    while (SERIAL_GPS.available() > 0)
    {
        gps.encode(
            static_cast<char>(
                SERIAL_GPS.read()
            )
        );
    }

    if (gps.location.isUpdated() &&
        gps.location.isValid())
    {
        gpsFix.latitude =
            gps.location.lat();

        gpsFix.longitude =
            gps.location.lng();

        gpsFix.updatedMs = millis();
        gpsFix.valid = true;
    }

    if (gps.satellites.isValid())
    {
        gpsFix.satellites =
            static_cast<uint8_t>(
                min(
                    static_cast<uint32_t>(254),
                    gps.satellites.value()
                )
            );
    }

    if (gps.hdop.isValid())
    {
        const uint32_t hdopX10 =
            static_cast<uint32_t>(
                (gps.hdop.value() + 5UL) / 10UL
            );

        gpsFix.hdopX10 =
            static_cast<uint8_t>(
                hdopX10 > 254UL
                    ? 254UL
                    : hdopX10
            );
    }

    if (gpsFix.valid &&
        millis() - gpsFix.updatedMs >
            GPS_STALE_MS)
    {
        gpsFix.valid = false;
    }
}

// =============================================================================
// PAYLOAD VERSION 4
// =============================================================================

static void buildPayload(
    uint8_t payload[34]
)
{
    size_t position = 0;

    const bool calm =
        ws80.valid &&
        ws80.wind <= 0.0f &&
        ws80.gust <= 0.0f;

    const float averageWind =
        calm
            ? 0.0f
            : (
                windStats.count > 0
                    ? windStats.averageWind()
                    : ws80.wind
            );

    const float maximumGust =
        calm
            ? 0.0f
            : (
                windStats.count > 0
                    ? windStats.maximumGust
                    : ws80.gust
            );

    const float direction =
        windStats.count > 0
            ? windStats.averageDirection(
                ws80.direction
            )
            : ws80.direction;

    // Byte 0: protocol version
    payload[position++] = 4;

    // Byte 1: flags
    payload[position++] =
        (ws80.valid ? 0x01 : 0x00) |
        (gpsFix.valid ? 0x02 : 0x00) |
        (!ws80.valid ? 0x10 : 0x00);

    // Bytes 2-3: average wind, m/s x 100
    putUint16(
        payload,
        position,
        ws80.valid
            ? scaleUnsigned16(
                averageWind,
                100.0f
            )
            : 0xFFFF
    );

    // Bytes 4-5: maximum gust, m/s x 100
    putUint16(
        payload,
        position,
        ws80.valid
            ? scaleUnsigned16(
                maximumGust,
                100.0f
            )
            : 0xFFFF
    );

    // Bytes 6-7: wind direction, degrees
    putUint16(
        payload,
        position,
        ws80.valid
            ? scaleUnsigned16(
                direction,
                1.0f
            )
            : 0xFFFF
    );

    // Bytes 8-9: temperature, C x 100
    int16_t temperature =
        isfinite(ws80.temperature)
            ? static_cast<int16_t>(
                roundf(
                    ws80.temperature * 100.0f
                )
            )
            : INT16_MIN;

    putUint16(
        payload,
        position,
        ws80.valid
            ? static_cast<uint16_t>(
                temperature
            )
            : 0x8000
    );

    // Bytes 10-11: rain reserved
    putUint16(
        payload,
        position,
        ws80.valid ? 0 : 0xFFFF
    );

    // Byte 12: humidity
    payload[position++] =
        ws80.valid
            ? ws80.humidity
            : 0xFF;

    // Byte 13: reserved
    payload[position++] = 0;

    // Bytes 14-15: WS80 battery, V x 1000
    putUint16(
        payload,
        position,
        ws80.valid
            ? scaleUnsigned16(
                ws80.battery,
                1000.0f
            )
            : 0xFFFF
    );

    // Bytes 16-19: light, lux
    putUint32(
        payload,
        position,
        ws80.valid
            ? ws80.light
            : 0xFFFFFFFFUL
    );

    // Bytes 20-21: UV x 100
    putUint16(
        payload,
        position,
        ws80.valid
            ? scaleUnsigned16(
                ws80.uv,
                100.0f
            )
            : 0xFFFF
    );

    // Bytes 22-23: T-Echo battery in mV
    putUint16(
        payload,
        position,
        readBoardBatteryMv()
    );

    // Bytes 24-27: latitude x 1,000,000
    const int32_t latitude =
        gpsFix.valid
            ? static_cast<int32_t>(
                llround(
                    gpsFix.latitude *
                    1000000.0
                )
            )
            : INT32_MIN;

    putUint32(
        payload,
        position,
        static_cast<uint32_t>(latitude)
    );

    // Bytes 28-31: longitude x 1,000,000
    const int32_t longitude =
        gpsFix.valid
            ? static_cast<int32_t>(
                llround(
                    gpsFix.longitude *
                    1000000.0
                )
            )
            : INT32_MIN;

    putUint32(
        payload,
        position,
        static_cast<uint32_t>(longitude)
    );

    // Byte 32: satellites
    payload[position++] =
        gpsFix.valid
            ? gpsFix.satellites
            : 0xFF;

    // Byte 33: HDOP x 10
    payload[position++] =
        gpsFix.valid
            ? gpsFix.hdopX10
            : 0xFF;
}

// =============================================================================
// E-PAPER DISPLAY
// =============================================================================

static void updateDisplay()
{
    if (!displayReady)
    {
        return;
    }

    display.setFullWindow();
    display.firstPage();

    do
    {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // Header
        display.fillRect(0, 0, 200, 25, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(&FreeMonoBold12pt7b);
        display.setCursor(50, 19);
        display.print("KWIND");

        // Main wind value
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeMonoBold18pt7b);
        display.setCursor(5, 61);

        if (ws80.valid)
        {
            const float average =
                windStats.count > 0
                    ? windStats.averageWind()
                    : ws80.wind;

            display.print(average, 1);
        }
        else
        {
            display.print("--.-");
        }

        display.setFont(&FreeMonoBold9pt7b);
        display.setCursor(137, 60);
        display.print("m/s");

        // Gust and direction boxes
        display.drawRoundRect(3, 69, 95, 37, 5, GxEPD_BLACK);
        display.drawRoundRect(102, 69, 95, 37, 5, GxEPD_BLACK);

        display.setFont(&FreeMono9pt7b);
        display.setCursor(10, 83);
        display.print("GUST");
        display.setCursor(109, 83);
        display.print("DIR");

        display.setFont(&FreeMonoBold9pt7b);
        display.setCursor(10, 101);

        if (ws80.valid)
        {
            const float gust =
                windStats.count > 0
                    ? windStats.maximumGust
                    : ws80.gust;

            display.print(gust, 1);
            display.print(" m/s");
        }
        else
        {
            display.print("--.-");
        }

        display.setCursor(109, 101);

        if (ws80.valid)
        {
            display.print(ws80.direction);
            display.print(" deg");
        }
        else
        {
            display.print("---");
        }

        // Coordinates
        display.setFont(nullptr);
        display.setTextSize(1);
        display.setCursor(5, 119);
        display.print("LAT ");

        if (gpsFix.valid)
        {
            display.print(gpsFix.latitude, 6);
        }
        else
        {
            display.print("waiting for GPS");
        }

        display.setCursor(5, 131);
        display.print("LON ");

        if (gpsFix.valid)
        {
            display.print(gpsFix.longitude, 6);
        }
        else
        {
            display.print("waiting for GPS");
        }

        display.drawFastHLine(3, 139, 194, GxEPD_BLACK);

        // Connection status
        display.setFont(&FreeMonoBold9pt7b);
        display.setCursor(5, 155);
        display.print(lorawanJoined ? "LORA OK" : "LORA OFF");

        display.setCursor(108, 155);
        display.print(gpsFix.valid ? "GPS OK" : "GPS WAIT");

        // Bottom information
        display.setFont(nullptr);
        display.setTextSize(1);
        display.setCursor(5, 172);
        display.print("SAT ");
        display.print(gpsFix.satellites);
        display.print("   SEND ");
        display.print(sendIntervalMs / 60000UL);
        display.print(" min");

        display.setCursor(5, 187);
        display.print("BAT ");
        display.print(readBoardBatteryMv());
        display.print("mV   DL ");

        if (lastDownlinkLength > 0)
        {
            display.print("F");
            display.print(lastDownlinkPort);
            display.print("/");
            display.print(lastDownlinkLength);
        }
        else
        {
            display.print("none");
        }
    }
    while (display.nextPage());

    // Put the panel into low-power mode after every refresh.
    display.hibernate();
}

static void initializeDisplay()
{
    pinMode(EPD_BACKLIGHT, OUTPUT);
    digitalWrite(EPD_BACKLIGHT, LOW);

    displaySpi = new SPIClass(
        NRF_SPIM2,
        EPD_MISO,
        EPD_SCLK,
        EPD_MOSI
    );

    displaySpi->begin();

    display.epd2.selectSPI(
        *displaySpi,
        SPISettings(
            4000000,
            MSBFIRST,
            SPI_MODE0
        )
    );

    display.init(0, true, 10, false);
    display.setRotation(3);
    displayReady = true;
    updateDisplay();
}

// =============================================================================
// DOWNLINK
// =============================================================================

static void processDownlink(
    const uint8_t* data,
    size_t length,
    uint8_t port
)
{
    lastDownlinkPort = port;
    lastDownlinkLength = length;

    SERIAL_MON.print("[LoRaWAN] Downlink FPort ");
    SERIAL_MON.print(port);
    SERIAL_MON.print(" (HEX): ");

    for (size_t i = 0; i < length; i++)
    {
        if (data[i] < 0x10)
        {
            SERIAL_MON.print('0');
        }

        SERIAL_MON.print(data[i], HEX);
    }

    SERIAL_MON.println();

    // Supported interval commands:
    //   02       -> two-minute interval
    //   01 02    -> command 0x01, two-minute interval
    uint8_t minutes = 0;

    if (length == 1)
    {
        minutes = data[0];
    }
    else if (length >= 2 && data[0] == 0x01)
    {
        minutes = data[1];
    }

    if (minutes >= 1 && minutes <= 60)
    {
        sendIntervalMs =
            static_cast<uint32_t>(minutes) *
            60000UL;

        lastSendMs = millis();

        SERIAL_MON.print(
            "[LoRaWAN] New interval: "
        );
        SERIAL_MON.print(minutes);
        SERIAL_MON.println(" minute(s)");
    }
    else if (length > 0)
    {
        SERIAL_MON.println(
            "[LoRaWAN] Downlink received but command is invalid"
        );
    }

}

// =============================================================================
// LORAWAN
// =============================================================================

static bool initializeRadio()
{
    SERIAL_MON.println(
        "[LoRaWAN] Initializing SPI..."
    );

    radioSpi = new SPIClass(
        NRF_SPIM3,
        LORA_MISO,
        LORA_SCLK,
        LORA_MOSI
    );

    radioSpi->begin();

    SPISettings radioSpiSettings;

    radio = new SX1262(
        new Module(
            LORA_CS,
            LORA_DIO1,
            LORA_RST,
            LORA_BUSY,
            *radioSpi,
            radioSpiSettings
        )
    );

    SERIAL_MON.println(
        "[LoRaWAN] Initializing SX1262..."
    );

    // The T-Echo uses a 1.8 V TCXO controlled by SX1262 DIO3.
    // false selects the SX1262 DC-DC regulator.
    int16_t state = radio->begin(
        868.0,
        125.0,
        9,
        7,
        0x34,
        22,
        8,
        1.8,
        false
    );

    SERIAL_MON.print(
        "[LoRaWAN] SX1262 result: "
    );
    SERIAL_MON.println(state);

    if (state != RADIOLIB_ERR_NONE)
    {
        return false;
    }

    // The T-Echo antenna switch is controlled by SX1262 DIO2.
    state = radio->setDio2AsRfSwitch(true);

    SERIAL_MON.print(
        "[LoRaWAN] RF switch result: "
    );
    SERIAL_MON.println(state);

    if (state != RADIOLIB_ERR_NONE)
    {
        return false;
    }

    state = radio->setCurrentLimit(80.0);

    if (state != RADIOLIB_ERR_NONE)
    {
        SERIAL_MON.print(
            "[LoRaWAN] Current limit failed: "
        );
        SERIAL_MON.println(state);
        return false;
    }

    lorawan = new LoRaWANNode(
        radio,
        &EU868
    );

    // Helium LoRaWAN 1.0.x:
    // NwkKey is NULL; AppKey is used.
    state = lorawan->beginOTAA(
        JOIN_EUI,
        DEV_EUI,
        nullptr,
        APP_KEY
    );

    if (state != RADIOLIB_ERR_NONE)
    {
        SERIAL_MON.print(
            "[LoRaWAN] OTAA setup failed: "
        );
        SERIAL_MON.println(state);

        return false;
    }

    return true;
}

static bool joinLorawan()
{
    if (lorawan == nullptr)
    {
        return false;
    }

    SERIAL_MON.println(
        "[LoRaWAN] Joining Helium EU868..."
    );

    const int16_t state =
        lorawan->activateOTAA();

    SERIAL_MON.print(
        "[LoRaWAN] Join result: "
    );
    SERIAL_MON.println(state);

    const bool joined =
        state == RADIOLIB_ERR_NONE ||
        state ==
            RADIOLIB_LORAWAN_NEW_SESSION ||
        state ==
            RADIOLIB_LORAWAN_SESSION_RESTORED;

    if (joined)
    {
        SERIAL_MON.println(
            "[LoRaWAN] Joined successfully"
        );
    }
    else
    {
        SERIAL_MON.println(
            "[LoRaWAN] Join failed"
        );
        SERIAL_MON.println(
            "[LoRaWAN] Retry in 60 seconds"
        );
    }

    return joined;
}

static bool sendPayload()
{
    if (lorawan == nullptr ||
        !lorawanJoined)
    {
        return false;
    }

    uint8_t payload[34];

    buildPayload(payload);

    SERIAL_MON.print("[LoRaWAN] Payload: ");

    for (size_t i = 0;
         i < sizeof(payload);
         i++)
    {
        if (payload[i] < 0x10)
        {
            SERIAL_MON.print('0');
        }

        SERIAL_MON.print(
            payload[i],
            HEX
        );
    }

    SERIAL_MON.println();

    uint8_t downlink[64] = {0};
    size_t downlinkLength = sizeof(downlink);
    LoRaWANEvent_t uplinkEvent = {};
    LoRaWANEvent_t downlinkEvent = {};

    const int16_t state =
        lorawan->sendReceive(
            payload,
            sizeof(payload),
            UPLINK_PORT,
            downlink,
            &downlinkLength,
            false,
            &uplinkEvent,
            &downlinkEvent
        );

    SERIAL_MON.print(
        "[LoRaWAN] Uplink result: "
    );
    SERIAL_MON.println(state);

    if (state >= RADIOLIB_ERR_NONE)
    {
        windStats.reset();

        if (state > 0)
        {
            SERIAL_MON.println(
                "[LoRaWAN] Downlink received"
            );

            processDownlink(
                downlink,
                downlinkLength,
                downlinkEvent.fPort
            );
        }
        else
        {
            SERIAL_MON.println(
                "[LoRaWAN] Uplink successful"
            );
        }

        updateDisplay();

        return true;
    }

    if (state ==
        RADIOLIB_ERR_NETWORK_NOT_JOINED)
    {
        lorawanJoined = false;
        lastJoinAttemptMs = millis();

        SERIAL_MON.println(
            "[LoRaWAN] Session lost"
        );
    }

    return false;
}

// =============================================================================
// STATUS
// =============================================================================

static void printStatus()
{
    SERIAL_MON.println();
    SERIAL_MON.println(
        "--------------------------------"
    );

    SERIAL_MON.print("LoRaWAN: ");
    SERIAL_MON.println(
        lorawanJoined
            ? "CONNECTED"
            : "NOT CONNECTED"
    );

    SERIAL_MON.print("WS80: ");
    SERIAL_MON.println(
        ws80.valid
            ? "OK"
            : "NO DATA"
    );

    if (ws80.valid)
    {
        SERIAL_MON.print("Wind: ");
        SERIAL_MON.print(ws80.wind, 2);
        SERIAL_MON.println(" m/s");

        SERIAL_MON.print("Gust: ");
        SERIAL_MON.print(ws80.gust, 2);
        SERIAL_MON.println(" m/s");

        SERIAL_MON.print("Direction: ");
        SERIAL_MON.print(ws80.direction);
        SERIAL_MON.println(" deg");

        SERIAL_MON.print("Temperature: ");
        SERIAL_MON.print(
            ws80.temperature,
            1
        );
        SERIAL_MON.println(" C");

        SERIAL_MON.print("Humidity: ");
        SERIAL_MON.print(ws80.humidity);
        SERIAL_MON.println(" %");

        SERIAL_MON.print("WS80 battery: ");
        SERIAL_MON.print(ws80.battery, 2);
        SERIAL_MON.println(" V");

        SERIAL_MON.print(
            "Buffered readings: "
        );
        SERIAL_MON.println(
            windStats.count
        );

        SERIAL_MON.print(
            "Buffered average: "
        );
        SERIAL_MON.print(
            windStats.averageWind(),
            2
        );
        SERIAL_MON.println(" m/s");

        SERIAL_MON.print(
            "Buffered maximum gust: "
        );
        SERIAL_MON.print(
            windStats.maximumGust,
            2
        );
        SERIAL_MON.println(" m/s");

        SERIAL_MON.print(
            "Buffered minimum wind: "
        );
        SERIAL_MON.print(
            windStats.minimumWind,
            2
        );
        SERIAL_MON.println(" m/s");
    }

    SERIAL_MON.print("GPS: ");
    SERIAL_MON.println(
        gpsFix.valid
            ? "FIX"
            : "WAITING"
    );

    SERIAL_MON.print("Satellites: ");
    SERIAL_MON.println(
        gpsFix.satellites
    );

    if (gpsFix.valid)
    {
        SERIAL_MON.print("Latitude: ");
        SERIAL_MON.println(
            gpsFix.latitude,
            6
        );

        SERIAL_MON.print("Longitude: ");
        SERIAL_MON.println(
            gpsFix.longitude,
            6
        );
    }

    SERIAL_MON.print(
        "T-Echo battery: "
    );
    SERIAL_MON.print(
        readBoardBatteryMv()
    );
    SERIAL_MON.println(" mV");

    SERIAL_MON.println(
        "--------------------------------"
    );
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
    SERIAL_MON.begin(115200);
    delay(2000);

    SERIAL_MON.println();
    SERIAL_MON.println(
        "================================"
    );
    SERIAL_MON.println(
        "KWind WS80 + LilyGO T-Echo"
    );
    SERIAL_MON.println(
        "GPS + Helium LoRaWAN EU868"
    );
    SERIAL_MON.println(
        "Payload protocol version 4"
    );
    SERIAL_MON.println(
        "================================"
    );

    // Enable T-Echo peripherals.
    pinMode(POWER_EN_PIN, OUTPUT);
    digitalWrite(POWER_EN_PIN, HIGH);

    delay(500);

    // Battery ADC.
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);

    // Discard first ADC reading.
    analogRead(ADC_PIN);

    // -------------------------------------------------------------------------
    // GPS
    // -------------------------------------------------------------------------

    pinMode(GPS_WAKEUP_PIN, OUTPUT);
    digitalWrite(GPS_WAKEUP_PIN, HIGH);

    pinMode(GPS_RESET_PIN, OUTPUT);
    digitalWrite(GPS_RESET_PIN, HIGH);

    SERIAL_GPS.setPins(
        GPS_RX_PIN,
        GPS_TX_PIN
    );

    SERIAL_GPS.begin(GPS_BAUD);

    SERIAL_MON.println(
        "[GPS] Serial2 started at 9600"
    );

    // -------------------------------------------------------------------------
    // WS80 UART
    // -------------------------------------------------------------------------

    // WS80 TX -> SDA/GPIO26.
    // SCL/GPIO27 is unused UART TX.
    //
    // Do not start Wire/I2C.
    // RX is the user-button signal. TX is unused; GPIO7 is used so
    // the UART does not conflict with the e-paper RESET pin (GPIO2).
    Serial1.setPins(42, 7);

   Serial1.begin(115200);

    SERIAL_MON.println(
        "[WS80] Serial1 started at 115200"
    );

    SERIAL_MON.println(
        "[WS80] TX -> user button GPIO42"
    );

    // -------------------------------------------------------------------------
    // E-PAPER DISPLAY
    // -------------------------------------------------------------------------

    SERIAL_MON.println(
        "[Display] Initializing e-paper..."
    );

    initializeDisplay();

    SERIAL_MON.println(
        "[Display] E-paper ready"
    );

    // -------------------------------------------------------------------------
    // LORAWAN
    // -------------------------------------------------------------------------

    if (initializeRadio())
    {
        lorawanJoined =
            joinLorawan();
    }
    else
    {
        SERIAL_MON.println(
            "[LoRaWAN] Fatal radio error"
        );
    }

    updateDisplay();

    lastJoinAttemptMs = millis();
    lastSendMs = millis();
    lastStatusMs = millis();

    SERIAL_MON.println(
        "[System] Setup finished"
    );
}

// =============================================================================
// LOOP
// =============================================================================

void loop()
{
    // Read GPS and WS80 continuously.
    pollGps();
    pollWs80();

    const uint32_t now = millis();

    // Retry joining every 60 seconds.
    if (lorawan != nullptr &&
        !lorawanJoined &&
        now - lastJoinAttemptMs >=
            JOIN_RETRY_MS)
    {
        lastJoinAttemptMs = now;

        lorawanJoined =
            joinLorawan();

        if (lorawanJoined)
        {
            lastSendMs = millis();
            updateDisplay();
        }
    }

    // Send payload every 60 seconds.
    if (lorawan != nullptr &&
        lorawanJoined &&
        now - lastSendMs >=
            sendIntervalMs)
    {
        lastSendMs = now;

        sendPayload();
    }

    // Print status every 10 seconds.
    if (now - lastStatusMs >=
        STATUS_INTERVAL_MS)
    {
        lastStatusMs = now;

        printStatus();
    }

    delay(5);
}
