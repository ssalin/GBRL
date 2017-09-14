//https://www.hackster.io/detox/send-mkr1000-data-to-google-sheets-1175ca
//by Stephen Borsay(Portland, OR, USA)
//Modified By Sam Salin for the GBRL

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
/**********/
/*Settings*/
/**********/
#define FILENAME "DATA.csv"
#define SAMPLE_MINS 10 //how many minutes to sample for
#define WIFIEN 1//change to 1 if wifi connection is used
const PROGMEM char* MY_SSID = "TEQUILA BATTERIES 2.4GHz";
#define SECURED 1 //change to 1 if the netwrok is secure
const PROGMEM char* MY_PWD = "HOOKERDICK69";   //""; //wifi password
#define DEV_NAME "DEV2"
#define LP 1 // change to 1 to use power saving code
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
WiFiClient client;  //Instantiate WiFi object
//pushingbox API stuff (might not need soon!)
const PROGMEM char WEBSITE[] = "api.pushingbox.com"; //pushingbox API server
const PROGMEM String devid = "v9606769D2CC718F"; //device ID on Pushingbox for our Scenario
//pubnub stuff
//const PROGMEM char pubkey[] = "Enter-your-pubkey";
//const PROGMEM char subkey[] = "Enter-your-subkey";
//const PROGMEM char channel[] = "channel";
int firstrun = 1; //used to write special things the first time through
//Set Up digital pin for Shinyei PM sensor
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
//sensor data variables
float Temp;
float Hum;
float PPM;
float Ozone;
float CO;
float rtctemp;
bool flag = 0; //used for checking return values. flag == true means things did not go well.
//make a SHT31 object
Adafruit_SHT31 sht31 = Adafruit_SHT31();

int status = WL_IDLE_STATUS; //global to avoid passing
/**************/
/**Functions**/
/*************/
void wakeup() {
  //Serial.println("Woke up!");
  //if (rtc.alarm1())
  //  LEDON();
  //if(rtc.alarm2())
  //  LEDOFF();
  rtc.alarm1();
  rtc.alarm2();
}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}

void Con2wifi() {
  while (status != WL_CONNECTED) {
    LEDON();
    delay (500);
    LEDOFF();
    delay (500);
    LEDON();
    Serial.print(F("Attempting to connect to Network: "));
    Serial.println(MY_SSID);

    if (SECURED)
      status = WiFi.begin(MY_SSID, MY_PWD); //connect to secured wifi
    else
      status = WiFi.begin(MY_SSID); //connect to open wifi
    delay(10000);  // wait 10 seconds for connection:
  }
  //printWifiStatus();
}

void LEDON() { //special debug stuff
  digitalWrite(A4, HIGH);
}
void LEDOFF() { //special debug stuff
  digitalWrite(A4, LOW);
}

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  starttime = millis();
  if (LP) //time to reprogram before sleeping
    delay(20000);
  pinMode(A4, OUTPUT); //special debug stuff
  LEDON();
  pinMode(PPMPIN, INPUT); // PPM
  pinMode(OZONEPIN, INPUT); // Ozone
  pinMode(RTC_IRQ, INPUT_PULLUP); // interrupts from RTC
  pinMode(COPIN, INPUT); // CO

  if (!sht31.begin(0x44))    // Set to 0x45 for alternate i2c addr
    Serial.println(F("Couldn't find SHT31"));

  Serial.println(F("starting RTC"));
  //set up RTC
  rtc.begin(RTC_CS);
  //rtc.autoTime(); //set time to compiler time. will be a little off but VERY close
  rtc.update(); //needs to be done to set alarms
  rtc.alarm1(); // clear alarm 1
  rtc.alarm2(); // clear alarm 2
  if (LP) {
    EIC->WAKEUP.reg |= (1 << RTC_IRQ); //SHOULD let interrupts wakeup from sleep
    attachInterrupt(RTC_IRQ, wakeup, LOW); //call wakeup when RTC_IRQ is low.
    rtc.enableAlarmInterrupt();
    interrupts();
  }
}

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
  return;
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
    //hard-coded because i dont know how to make dynamic arrays nicely outside of assembly, which is GROSS
    data[0] = Temp;
    data[1] = Hum;
    data[2] = PPM;
    data[3] = Ozone;
    data[4] = CO;

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
//but it may get changed to azure/AWS/similar/
//need to add a spot for RTC temp.
int postdata() {
  Serial.println(F("\nSending Data to Server..."));
  //API service using WiFi Client through PushingBox then relayed to Google
  if (client.connect(WEBSITE, 80)) {
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
    Serial.println(F("\nData Sent"));
    return 0;
  }

  else{
    Serial.println(F("ERROR CONNECTING TO SERVER"));
    return 1;
  }
}
int tmp;
//main
void loop() {
  //read temp
  LEDON();
  Temp = sht31.readTemperature();
  rtctemp = rtc.temperature(); //the RTC keeps track of temperature too apparently
  Serial.println(rtctemp);
  Hum = sht31.readHumidity(); //read humidity
  //get particulate information
  PPM = getppm();//concentration;
  Ozone = analogRead(OZONEPIN); //Read value from ozone pin
  //Ozone = 385 - (Ozone / 2); //ozone w/ a calibration curve
  CO = analogRead(COPIN); //carbon monoxide
  //printdata();
  //logdata();
  if (WIFIEN) {
    flag = 1;
    while(flag){
      Con2wifi();
      LEDOFF();
      delay(1000);
      LEDON();
      delay(1000);
      LEDOFF();
      delay(1000);
      LEDON();
      flag = postdata(); //will be set to 0 when connection goes through.
      LEDOFF();
      delay(5000);
      LEDON();
      WiFi.disconnect();
      status = WL_IDLE_STATUS;
    }
  }
  //go back to sleep
  if (LP) { //low power mode
    Serial.println(F("boutta go to sleep"));
    LEDOFF();
    tmp = 0;
    rtc.update(); //fix no wakeup?
    tmp = 60-rtc.minute(); // use tmp to figure out if the hour changes
    if(tmp > SAMPLE_MINS)
      rtc.setAlarm1(rtc.second(), rtc.minute() + SAMPLE_MINS); //alarm1 triggered when minute increments by 1; //alarm1 triggered when minute increments by 5
    else
      rtc.setAlarm1(rtc.second(),SAMPLE_MINS - tmp);
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; //turn on deep sleep SAMD board specific.
    __DSB(); //finish memory stuff
    //interrupts(); //maybe fixes light staying on?
    __WFI(); //wait for interrupt
    //noInterrupts(); //maybe fixes light staying on?
  }
  else{
    rtc.setAlarm1(rtc.second(), rtc.minute() + SAMPLE_MINS); 
    while (!rtc.alarm1()){
      delay (100);
    }
  }
}
