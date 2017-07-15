/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// #include <Wire.h>
// #include "rgb_lcd.h"

#define trace 1

// #ifdef trace
// #   define Serial.println(a) (Serial.println(a))
// #   define trace(a) (Serial.print(a))
// #else
// #   define Serial.println(a)
// #   define trace(a)
// #endif

int statusLed = 17;
bool joined = false;
bool movementDetected = true;
bool sending = false;
volatile byte detected = LOW;

// static rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

#define PIR_MOTION_SENSOR 8 //Use pin 2 to receive the signal from the module
#define LED A0              //the Grove - LED is connected to D4 of Arduino

//https://cdn-learn.adafruit.com/assets/assets/000/041/538/original/feather_32u4_LoRa_V1_2.png?1494120963

//http://forum.arduino.cc/index.php?topic=129535.0
#ifdef ARDUINO_ARCH_AVR

static const u1_t PROGMEM DEVEUI[8] = {0x39, 0x99, 0x16, 0xDE, 0x5A, 0x44, 0xDB, 0x00};
static const u1_t PROGMEM APPEUI[8] = {0x43, 0x4B, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70};
static const u1_t PROGMEM APPKEY[16] = {0x41, 0x11, 0x49, 0x51, 0x93, 0xD9, 0x9E, 0xEA, 0x9B, 0x70, 0x3A, 0xA8, 0x66, 0x2E, 0x4D, 0x61};

const lmic_pinmap lmic_pins = {
    .nss = 8, // CS pin on feather 32u4
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,                       //rst on feather 32u4
    .dio = {7, 1, LMIC_UNUSED_PIN}, //pin 1 jumpered to radio dio01
};

#endif

#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>

static const u1_t PROGMEM DEVEUI[8] = {0x8F, 0xCC, 0xB9, 0x59, 0xCC, 0xCB, 0x33, 0x00};                                                  //LSB
static const u1_t PROGMEM APPEUI[8] = {0x43, 0x4B, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70};                                                  // LSB
static const u1_t PROGMEM APPKEY[16] = {0xAB, 0x7E, 0x0D, 0x74, 0x4F, 0x0A, 0xC7, 0xA6, 0xE5, 0x44, 0xDB, 0x6B, 0x78, 0x7D, 0x5C, 0x7A}; //MSB

const lmic_pinmap lmic_pins = {
    // NodeMcu with RFM95W module
    .nss = D8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = D3,
    .dio = {D1, D2, LMIC_UNUSED_PIN},
};

#endif

#ifdef ARDUINO_ARCH_SAMD

static const u1_t PROGMEM DEVEUI[8] = { 0x77, 0x5A, 0x3B, 0x77, 0x16, 0xB6, 0xFD, 0x00 };                                                  //LSB
static const u1_t PROGMEM APPEUI[8] = { 0x43, 0x4B, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };                                                  // LSB
static const u1_t PROGMEM APPKEY[16] = { 0x56, 0x61, 0xE7, 0x3A, 0x4D, 0x4B, 0x9A, 0x2A, 0x66, 0x86, 0xD2, 0x79, 0x47, 0xDD, 0x6B, 0x6D }; //MSB

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,
    .dio = {2, 5, 6},
};

#endif

void os_getArtEui(u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint16_t count = 0;
static uint8_t mydata[2];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5;

void onEvent(ev_t ev)
{
    // trace(os_getTime());
    // trace(": ");
    switch (ev)
    {
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        // Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        digitalWrite(statusLed, HIGH);
        Serial.println(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("artKey: ");
            for (int i = 0; i < sizeof(artKey); ++i)
            {
                Serial.print(artKey[i], HEX);
            }
            Serial.println("");
            Serial.print("nwkKey: ");
            for (int i = 0; i < sizeof(nwkKey); ++i)
            {
                Serial.print(nwkKey[i], HEX);
            }
            Serial.println("");
            LMIC_setSeqnoUp(140);
        }
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        joined = true;
        sending = false;
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void do_send(osjob_t *j)
{

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        // Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        if (count > 100)
        {
            count = 0;
        }
        mydata[0] = count >> 8; // data in big-endian
        mydata[1] = (uint8_t)count;
        LMIC_setTxData2(1, mydata, 2, 0);
        // Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{

#ifdef ARDUINO_ARCH_ESP8266
    WiFi.mode(WIFI_OFF);
#endif
    pinMode(statusLed, OUTPUT);
    digitalWrite(statusLed, LOW);

    //  #if trace
    Serial.begin(9600);
    //  #endif
    Serial.println(F("Starting"));

#ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    //    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF10,23);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    pinMode(LED, OUTPUT);
    pinMode(PIR_MOTION_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIR_MOTION_SENSOR), movement, CHANGE);

    // lcd.begin(16, 2);

    // lcd.setRGB(colorR, colorG, colorB);

    // // Print a message to the LCD.
    // lcd.print("Connecting");
}

void loop()
{
    os_runloop_once();
}

void movement()
{
    count+=2;

    detected = digitalRead(PIR_MOTION_SENSOR);
    digitalWrite(LED, detected);

    // if (detected && joined && !sending)
    if (joined && !sending)
    {        
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        sending = true;
    }
}
