//https://www.hackster.io/detox/send-mkr1000-data-to-google-sheets-1175ca
//by Stephen Borsay(Portland, OR, USA)
//Since Arduino can't https, we need to use Pushingbox API (uses http)to run
//the Google Script (uses https). Alternatly use Ivan's SecureWifi encryption
//Modified By Sam Salin for the GBRL
//Arduino Actually Can HTTPS, hoping to modify the code to use it
//PIN INFORMATION
//  PPM   -> MKR_0
//  CO    -> A3 (18)
//  Ozone -> A1 (16)
//  SHT31 -> iic (SCL/SDA)
//  CO2   -> iic (SCL/SDA)
//  RTC CS-> 6
//  SD CS -> 7
//  RTC alarm innterupts will come on A2

/************/
/**Includes**/
/************/
#include <WiFi101.h>
#include <SPI.h> //SPI lib, RTC/SD card
#include <SD.h>  // SD lib
#include "Adafruit_SHT31.h" //temp/RH sensor lib
#include <SparkFunDS3234RTC.h> //rtc lib
<<<<<<< HEAD
//#include "ArduinoLowPower.h"
/**********/
/*Settings*/
/**********/
#define SAMPLE_RESOLUTION 10000//200000//posts about every 17 mins? ////300000 5 mins, posts every 17? //600000 10 mins posts every 40??
#define FILENAME "DATA.csv"
#define WIFIEN 1//change to 1 if wifi connection is used
const PROGMEM char* MY_SSID = "PSU"; //"TEQUILA BATTERIES 2.4GHz";
#define SECURED 0 //change to 1 if the netwrok is secure
const PROGMEM char* MY_PWD =  "";//"HOOKERDICK69";   //""; //wifi password
#define DEV_NAME "DEV1"
#define LP 0 // change to 1 to use power saving code
#define SDP 1 //change to 1 if there is an SD card present
#define DEBUG 1 // change to 1 to not print to serial
=======
//#include <ArduinoLowPower.h>
/**********/
/*Settings*/
/**********/
#define SAMPLE_RESOLUTION 600000 //how often device will gather + report sensor data. right now it's every 10 minutes!
#define FILENAME "DATA.csv"
#define WIFIEN 1//change to 1 if wifi connection is used
const PROGMEM char* MY_SSID = "TEQUILA BATTERIES 2.4GHz";//"PSU";
#define SECURED 1 //change to 1 if the netwrok is secure
const PROGMEM char* MY_PWD =  "HOOKERDICK69";   //""; //wifi password
#define DEV_NAME "DEV2"
#define LP 0 // change to 1 to use power saving code
>>>>>>> origin/master
/***************/
/**Definitions**/
/***************/
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define SD_CS 7//chip select for SD card
#define RTC_CS 6 //chip select for RTC
#define RTC_IRQ A2 // A2 connected to SQW of RTC
#define PRINT_USA_DATE //October 31, 2016: 10/31/16 vs. 31/10/16
#define COPIN A3 //analogue 3
#define OZONEPIN A1 //analogue 1
#define PPMPIN  0 //MKR_0
/***********/
/**Globals**/
/***********/
//pushingbox API stuff
const PROGMEM char WEBSITE[] = "api.pushingbox.com"; //pushingbox API server
const PROGMEM String devid = "v9606769D2CC718F"; //device ID on Pushingbox for our Scenario

//Define analog input and values for for Mics 03 sensor:
int OzoneMSensorValue = 0;
int firstrun = 1; //used to write special things the first time through
//Set Up digital pin for Shinyei PM sensor
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

float Temp;
float Hum;
float PPM;
float Ozone; 
float CO;

//make a SHT31 object
Adafruit_SHT31 sht31 = Adafruit_SHT31();

int status = WL_IDLE_STATUS; //global to avoid passing

/**************/
/**Functions**/
/*************/
<<<<<<< HEAD
void wakeup(){
  Serial.begin(9600); 
  Serial.println("Woke up!");
} 
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
=======
void wakeup(){ 
  //empty funcion
  //because IRQ handler needs a function
  //but we just wake up on IRQ, nothing special
} 
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
>>>>>>> origin/master
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
<<<<<<< HEAD
  Serial.print(F("IP Address: "));
=======
  Serial.print("IP Address: ");
>>>>>>> origin/master
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}
void Con2wifi(){
  while (status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to Network: "));
    Serial.println(MY_SSID);
    
    if(SECURED)
      status = WiFi.begin(MY_SSID, MY_PWD); //connect to secured wifi
    else
      status = WiFi.begin(MY_SSID); //connect to open wifi
   delay(10000);  // wait 10 seconds for connection:
  }
<<<<<<< HEAD
 printWifiStatus();
=======
>>>>>>> origin/master
}
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  starttime = millis();
  delay(5000);
  //while (!Serial){ }
<<<<<<< HEAD
  Serial.println(F("serial conection initialized"));
=======
  Serial.println("serial conection initialized");
>>>>>>> origin/master
  pinMode(PPMPIN, INPUT); // PPM
  pinMode(OZONEPIN, INPUT); // Ozone
  pinMode(RTC_IRQ, INPUT_PULLUP); // interrupts from RTC
  pinMode(COPIN, INPUT); // CO
  
  if (!sht31.begin(0x44))    // Set to 0x45 for alternate i2c addr
    Serial.println(F("Couldn't find SHT31"));
<<<<<<< HEAD
    
=======
  
  //Serial.println(F("starting SD"));
  //if (!SD.begin(SD_CS)) 
  //  Serial.println(F("PROBLEM WITH SD CARD"));
>>>>>>> origin/master
  Serial.println(F("starting RTC"));
  //else Serial.println("found SHT31");

  //set up RTC
  rtc.begin(RTC_CS);
  //rtc.autoTime(); //set time to compiler time. will be a little off but VERY close
  rtc.update(); //needs to be done to set alarms
<<<<<<< HEAD
  if(LP){
    LowPower.attachInterruptWakeup(RTC_IRQ, wakeup, FALLING);
    rtc.enableAlarmInterrupt();
  }
}

//this code is only used by K-30 C02 sensor.
=======
  //rtc.enableAlarmInterrupt();
  //rtc.setAlarm1(30); //alarm1 alert when seconds hits 30
  //rtc.setAlarm1(rtc.minute() + 5); //alarm1 triggered when minute increments by 5
}
//this code is only used by K-30 C02 sensor.
//later i will make a different version of the code for with/without CO2
//because it takes a up a lot of space and is ugly.
//alternatively ifdef it out
>>>>>>> origin/master
int readCO2() {
  int co2_value = 0;  // Store the CO2 value inside this variable.
  digitalWrite(13, HIGH);  // turn on LED
  // On most Arduino platforms this pin is used as an indicator light.
  //////////////////////////
  /* Begin Write Sequence */
  //////////////////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.write(0x2A);
  Wire.endTransmission();
  /////////////////////////
  /* End Write Sequence. */
  /////////////////////////
  /*
    Wait 10ms for the sensor to process our command. The sensors's
    primary duties are to accurately measure CO2 values. Waiting 10ms
    ensures the data is properly written to RAM
  */
  delay(10);
  /////////////////////////
  /* Begin Read Sequence */
  /////////////////////////
  /*
    Since we requested 2 bytes from the sensor we must read in 4 bytes.
    This includes the payload, checksum, and command status byte.
  */
  Wire.requestFrom(0x68, 4);
  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};
  /*
    Wire.available() is not necessary. Implementation is obscure but we
    leave it in here for portability and to future proof our code
  */
  while (Wire.available())
  {
    buffer[i] = Wire.read();
    i++;
  }
  ///////////////////////
  /* End Read Sequence */
  ///////////////////////
  /*
    Using some bitwise manipulation we will shift our buffer
    into an integer for general consumption
  */
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

float getppm() {
  //Read Particle Count from PM sensor
  duration = pulseIn(PPMPIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  if ((millis() - starttime) > sampletime_ms) {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    //Serial.print(lowpulseoccupancy);
    //Serial.print(",");
    //Serial.print(ratio);
    //Serial.print(",");
    //Serial.println(concentration);
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  return concentration;
}

void printdata() {
  rtc.update();
  String htime = String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second());
  String hdate = String(rtc.month()) + "/" + String(rtc.date()) + "/" + String(rtc.year()) + " - ";
  Serial.println(hdate + htime);
  Serial.print(F("Temperature: "));
  Serial.print(Temp);
  Serial.println(F(" *C"));
  Serial.print(F("Humidity: "));
  Serial.print(Hum);
  Serial.println(F("%"));
  Serial.print("PPM:  ");
  Serial.println(PPM);
  Serial.print(F("Ozone Concentration: "));
  Serial.println(Ozone);
  Serial.print(F("CO Concentration: "));
  Serial.println(CO);
  Serial.println();
}

//function to write data to SD card.
int logdata() {
  
  SD.begin(SD_CS); //for some reason this fixed it only writing once.
  //string that gets written to SD card
  String WriteMe = "";

  //open file
  File W_File = SD.open(FILENAME, FILE_WRITE); // write file, if that wasnt clear
  if (W_File) {

    //prepping data for ease of appending to WriteMe
    float data[5]; //should really use a #define instead of hardcoded 5. somthing like NUM_SENSORS
<<<<<<< HEAD
    //hard-coded because i dont know how to make dynamic arrays nicely outside of assembly, which is GROSS
=======
    //hard-coded because i dont know how to make dynamic arrays nicely
>>>>>>> origin/master
    data[0] = Temp;
    data[1] = Hum;
    data[2] = PPM;
    data[3] = Ozone;
    data[4] = CO;
<<<<<<< HEAD
    
=======

    //WriteMe += hdate + htime; // timestamp at beginning
>>>>>>> origin/master
    //get time
    rtc.update();
    String htime = String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second());
    String hdate = String(rtc.month()) + "/" + String(rtc.date()) + "/" + String(rtc.year());

    if (firstrun) {
      firstrun = 0; //the global
      W_File.println(F("Beginning of this data block"));
      W_File.println(F("Date,Time,Temperature(Celcius),Humidity(%),Particles(pcs/.01 cubic feet),Ozone(PPB),CO(PPM)"));
      W_File.println();
    }
    W_File.print(hdate + ","); //make nice CSV's
    W_File.print(htime + ",");//make nice CSV's
    for (int i = 0; i < 5; ++i) {
      W_File.print(String(data[i]));
      W_File.flush();
      //WriteMe += String(data[i]);
      if (i != 4) //no delimeter after final data item
        W_File.print(",");
    }
    W_File.println();
    W_File.close();
  }


  else {
    Serial.println(F("error opening file"));
    return 1; //
  }

  return 0; //no return = good
}

//function for sending data to cloud. right now it uses google sheets
//but it may get changed to azure since sheets isnt very elegant
int postdata() {
  Con2wifi();
<<<<<<< HEAD
  Serial.println(F("\nSending Data to Server...")); 
=======
  Serial.println("\nSending Data to Server..."); 
>>>>>>> origin/master
    // if you get a connection, report back via serial:
  WiFiClient client;  //Instantiate WiFi object, can scope from here or Globally
    //API service using WiFi Client through PushingBox then relayed to Google
    if (client.connect(WEBSITE, 80)){ 
         client.print("GET /pushingbox?devid=" + devid
       + "&humidityData=" + (String) Temp
       + "&celData="      + (String) Hum
       + "&fehrData="     + (String) PPM
       + "&hicData="      + (String) Ozone
       + "&hifData="      + (String) CO
       + "&name="         + (String) DEV_NAME
         );

      client.println(" HTTP/1.1"); 
      client.print("Host: ");
      client.println(WEBSITE);
      client.println("User-Agent: MKR1000/1.0");
      client.println();
      client.stop();
<<<<<<< HEAD
      Serial.println(F("\nData Sent")); 
=======
      Serial.println("\nData Sent"); 
>>>>>>> origin/master
    }
   
    else
      Serial.println(F("ERROR CONNECTING TO SERVER"));
 
    WiFi.disconnect();
    status = WL_IDLE_STATUS;    
}

//main
<<<<<<< HEAD
void loop() {  
  //read temp
  Temp = sht31.readTemperature();
  float rtctemp = rtc.temperature(); //the RTC keeps track of temperature too apparently
  Serial.println(rtctemp);
=======
void loop() {
  if(LP){
    //power management fun
    //attachInterrupt(0, wakeup, CHANGE);
    //Serial.println("boutta go to sleep");
    //rtc.setAlarm1(rtc.minute() + 1); //alarm1 triggered when minute increments by 5
    //LowPower.sleep();
    //Serial.println("Woke up!");
    //detachInterrupt(0); 
  }
  else
    delay(SAMPLE_RESOLUTION);
  
  //read temp
  Temp = sht31.readTemperature();
>>>>>>> origin/master
  //read humidity
  Hum = sht31.readHumidity();
  //get ppm information
  PPM = getppm();//concentration;
  //read O3
  Ozone = analogRead(OZONEPIN); //Read value from ozone pin
  //Ozone = 385 - (Ozone / 2); //ozone w/ a calibration curve
  //read CO
  CO = analogRead(COPIN);
<<<<<<< HEAD
  printdata();
  logdata();
  if(WIFIEN)
    postdata();
    
  //go back to sleep  
  if(LP){ //low power mode
    Serial.println(F("boutta go to sleep"));
    rtc.update(); //fix no wakeup?
    rtc.setAlarm1(rtc.minute() + 1); //alarm1 triggered when minute increments by 5
    //attatch interupt
    interrupts(); //turn interrupts on
    //LowPower.sleep();
  }
  else //not low power mode
    delay(SAMPLE_RESOLUTION);
  
}
=======

  printdata();
  logdata();
  
  if(WIFIEN)
    postdata();
  //go back to sleep
  //interrupts(); //turn interrupts back on
}
>>>>>>> origin/master
