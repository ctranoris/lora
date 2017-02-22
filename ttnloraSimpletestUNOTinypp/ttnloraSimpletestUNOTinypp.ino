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
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const PROGMEM u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static const PROGMEM u1_t NWKSKEY[16] = { 0x3D, 0xC6, 0x62, 0x23, 0x79, 0xFA, 0xDE, 0x22, 0x09, 0xDC, 0xF6, 0x89, 0x72, 0x5E, 0x50, 0x21 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static const u1_t PROGMEM APPSKEY[16] = { 0x90, 0x8E, 0x05, 0x6A, 0xA9, 0x80, 0x2B, 0x0E, 0x42, 0xED, 0xC1, 0xDA, 0x13, 0xB9, 0x52, 0x96 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x26011831; // <-- Change this address for every node!


String message = "HELLO AGRODU01";

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5; //60

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};


#define ss Serial
byte dataoutgoing[50];

static uint8_t mydata[] = "Hello: ";
static osjob_t sendjob;
int counter = 100;

void onEvent (ev_t ev) {
    ss.print(os_getTime());
    ss.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            ss.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            ss.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            ss.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            ss.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            ss.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            ss.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            ss.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            ss.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            ss.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            ss.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                ss.print(F("Data Received: "));
                ss.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                ss.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            ss.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            ss.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            ss.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            ss.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            ss.println(F("EV_LINK_ALIVE"));
            break;
         default:
            ss.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){

    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

     
    ss.println("LAT = no newData.");
     
    
    //message = "M"+String(counter) + message;
    message.getBytes(dataoutgoing, message.length()+1);
    counter++;
    if (counter>=999){
      counter = 100;
    }
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        ss.println(F("OP_TXRXPEND, not sending"));
    } else {

        LMIC_setTxData2(1, (uint8_t*) dataoutgoing, message.length() , 0);
        ss.println("Sending msg: "+message);
        ss.println(F("Packet queued: "));
        for (int i = 0; i < message.length(); i++) {
                ss.print((char)dataoutgoing[i]);
        }
        ss.println(F("->"));        
      digitalWrite( 8, HIGH);
      delay(500);              // wait for a second
      digitalWrite( 8, LOW);      
        
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    pinMode( 8, OUTPUT); 
    
    Serial.begin(9600);    
    ss.begin(9600);
    ss.println(F("Starting"));


    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

