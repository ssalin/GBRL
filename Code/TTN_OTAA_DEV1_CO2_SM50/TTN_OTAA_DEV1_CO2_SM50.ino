#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SD.h>  // SD lib
#include "Adafruit_SHT31.h"     //temp/RH sensor lib
#include <SparkFunDS3234RTC.h>  //rtc lib
/****************/
/***PINMAPPING***/
/****************/
#define DEBUG 1 //give information over serial
#define LORA_DIO0 1 //lora IRQ pin, IRQs can come here.
#define LORA_DIO1 2 // needed for LoRa Mode
#define LORA_DIO2 3 //needed for FSK?, can be UNDEFINED_PIN probably
#define LORA_CS   4  //LORA CS PIN
#define LORA_RST  5
#define SD_CS 7//chip select for SD card
#define RTC_CS 6 //chip select for RTC
#define RTC_IRQ A2 // A2 connected to SQW of RTC
#define PRINT_USA_DATE //October 31, 2016: 10/31/16 vs. 31/10/16
//#define COPIN A3 //analogue 3
//#define PPMPIN  0 //MKR_0
#define OZONEPIN A6
#define FILENAME "DATA.csv" //name of data file

//special data structure for sending
typedef union
{
 float val;
 uint8_t bytes[4];
} FLOATUNION_t;
//
/***********/
/**Globals**/
/***********/
Adafruit_SHT31 sht31 = Adafruit_SHT31();
FLOATUNION_t Temp;
FLOATUNION_t Hum;
//FLOATUNION_t PPM;
FLOATUNION_t Ozone;
//FLOATUNION_t CO;
FLOATUNION_t CO2;
FLOATUNION_t RTCTEMP;
FLOATUNION_t COUNT;
bool JOINSTATUS = 0;

//SPECIAL SHINYEI STUFF
//Set Up digital pin for Shinyei PM sensor
/*unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
*///


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x76, 0x7E, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x95, 0x35, 0x37, 0xD5, 0xEC, 0xC2, 0xE9, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format
static const u1_t PROGMEM APPKEY[16] = { 0xC4, 0x69, 0x7D, 0xEC, 0x2E, 0x67, 0x6D, 0x81, 0x1D, 0x09, 0x01, 0xA1, 0x7A, 0xD8, 0x20, 0x0E };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,//LORA_RST, //not strictly needed
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            JOINSTATUS = 1;
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            JOINSTATUS = 0;
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            JOINSTATUS = 0;
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(10000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    COUNT.val = 1;
    //pinMode(PPMPIN, INPUT); // PPM
   // PPM.val = 1000;
    pinMode(OZONEPIN, INPUT); // Ozone  
    pinMode(RTC_IRQ, INPUT_PULLUP); // interrupts from RTC
    //pinMode(COPIN, INPUT); // CO
    rtc.begin(RTC_CS);
    //rtc.autoTime(); //set time to compiler time. will be a little off but VERY close
    rtc.update(); //needs to be done to set alarms
    rtc.alarm1(); // clear alarm 1 if it's already triggered (power loss for a while)
    rtc.alarm2(); // same as above but alarm 2


    if (!sht31.begin(0x44))    // Set to 0x45 for alternate i2c addr
      Serial.println(F("Couldn't find SHT31"));
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}
/*
float getppm() {
  //Read Particle Count from PM sensor
  duration = pulseIn(PPMPIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  if ((millis() - starttime) > sampletime_ms) {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  return concentration;
}
*/
void printdata() {
  rtc.update();
  String htime = String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second());
  String hdate = String(rtc.month()) + "/" + String(rtc.date()) + "/" + String(rtc.year()) + " - ";
  Serial.println(hdate + htime);
  Serial.print(F("Temperature: "));
  Serial.print(Temp.val);
  Serial.println(F(" *C"));
  Serial.print(F("RTC Temperature: " ));
  Serial.print(RTCTEMP.val);
  Serial.println(F(" *C"));
  Serial.print(F("Humidity: "));
  Serial.print(Hum.val);
  Serial.println(F("%"));
//  Serial.print("PPM:  ");
//  Serial.println(PPM.val);
  Serial.print(F("Ozone Concentration: "));
  Serial.println(Ozone.val);
//  Serial.print(F("CO Concentration: "));
//  Serial.println(CO.val);
//  Serial.print(F("CO2 Concentration: "));
  Serial.println(CO2.val);
  Serial.print(F("Packet # "));
  Serial.println(COUNT.val);
  Serial.println();  
  return;
}

float readCO2() 
{ 
  int co2_value = 0;  // Store the CO2 value inside this variable. 
  int co2Addr = 0x68;
  Wire.beginTransmission(co2Addr); 
  Wire.write(0x22); 
  Wire.write(0x00); 
  Wire.write(0x08); 
  Wire.write(0x2A); 
  Wire.endTransmission(); 
  delay(10); 
  Wire.requestFrom(co2Addr, 4); 
  byte i = 0; 
  byte buffer[4] = {0, 0, 0, 0}; 
  while (Wire.available()) 
  { 
    buffer[i] = Wire.read(); 
    i++; 
  } 
  co2_value = 0; 
  co2_value |= buffer[1] & 0xFF; 
  co2_value = co2_value << 8; 
  co2_value |= buffer[2] & 0xFF; 
  byte sum = 0; //Checksum Byte 
  sum = buffer[0] + buffer[1] + buffer[2]; //Byte addition utilizes overflow 
  if (sum == buffer[3]) 
  { 
    // Success! 
    digitalWrite(13, LOW); 
    return co2_value; 
  } 
  else 
  { 
    // Failure! 
    /* 
      Checksum failure can be due to a number of factors, 
      fuzzy electrons, sensor busy, etc. 
    */ 
    digitalWrite(13, LOW); 
    return 0; 
  } 
}

void getdata(){
  Temp.val = sht31.readTemperature();
  RTCTEMP.val = rtc.temperature(); //the RTC keeps track of temperature too apparently
  Hum.val = sht31.readHumidity(); //read humidity
  //PPM.val = getppm();//concentration;
  Ozone.val = analogRead(OZONEPIN); //Read value from ozone pin
  //CO.val = analogRead(COPIN); //carbon monoxide
  CO2.val=readCO2();
}

void senddata(){
     uint8_t buf [24]; //buffer for sent data
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        printdata();
        // Prepare upstream data transmission at the next possible time.
        buf[0] = Temp.bytes[0];
        buf[1] = Temp.bytes[1];
        buf[2] = Temp.bytes[2];
        buf[3] = Temp.bytes[3];
        //rtctemp
        buf[4] = RTCTEMP.bytes[0];
        buf[5] = RTCTEMP.bytes[1];
        buf[6] = RTCTEMP.bytes[2];
        buf[7] = RTCTEMP.bytes[3];
        //hum
        buf[8] = Hum.bytes[0];
        buf[9] = Hum.bytes[1];
        buf[10] = Hum.bytes[2];
        buf[11] = Hum.bytes[3];
        //ppm
        //buf[12] = PPM.bytes[0];
        //buf[13] = PPM.bytes[1];
        //buf[14] = PPM.bytes[2];
        //buf[15] = PPM.bytes[3];
        //ozone
        buf[12] = Ozone.bytes[0];//buf[16] = Ozone.bytes[0];//
        buf[13] = Ozone.bytes[1];//buf[17] = Ozone.bytes[1];//
        buf[14] = Ozone.bytes[2];//buf[18] = Ozone.bytes[2];//
        buf[15] = Ozone.bytes[3];//buf[19] = Ozone.bytes[3];//
        //co
        //buf[20] = CO.bytes[0];
        //buf[21] = CO.bytes[1];
        //buf[22] = CO.bytes[2];
        //buf[23] = CO.bytes[3];
        //sent counter
        buf[16] = COUNT.bytes[0];//buf[24] = COUNT.bytes[0];//
        buf[17] = COUNT.bytes[1];//buf[25] = COUNT.bytes[1];//
        buf[18] = COUNT.bytes[2];//buf[26] = COUNT.bytes[2];//
        buf[19] = COUNT.bytes[3];//buf[27] = COUNT.bytes[3];//
        //
        buf[20] = CO2.bytes[0];//buf[28] = CO2.bytes[0];//
        buf[21] = CO2.bytes[1];//buf[29] = CO2.bytes[1];
        buf[22] = CO2.bytes[2];//buf[30] = CO2.bytes[2];
        buf[23] = CO2.bytes[3];//buf[31] = CO2.bytes[3];

        
        LMIC_setTxData2(1, buf, sizeof(buf), 0);
        Serial.println(F("Packet queued"));
        ++COUNT.val;
        delay(2000);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void loop() { 
  if(JOINSTATUS){
    getdata();
    senddata();
    delay(1000);
  }
  os_runloop_once();
}

