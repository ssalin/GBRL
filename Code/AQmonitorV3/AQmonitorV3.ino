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
#include<SPI.h> //SPI lib, RTC/SD card
#include<SD.h>  // SD lib
#include "Adafruit_SHT31.h" //temp/RH sensor lib
#include <SparkFunDS3234RTC.h> //rtc lib
#include <sleep.h>

/***************/
/**Definitions**/
/***************/
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define SAMPLE_RESOLUTION 5000 //how often device will gather + report sensor data
#define SD_CS 7//chip select for SD card
#define RTC_CS 6 //chip select for RTC
#define RTC_IRQ A2 // A2 connected to SQW of RTC
#define IRQ_PIN A2 //analogue 2
#define PRINT_USA_DATE //October 31, 2016: 10/31/16 vs. 31/10/16
#define FILENAME "DATA.csv"
#define COPIN A3 //analogue 3
#define OZONEPIN A1 //analogue 1
#define PPMPIN  0 //MKR_0
#define DEV_NAME "DEV2"

/***********/
/**Globals**/
/***********/
//pushingbox API stuff
const char WEBSITE[] = "api.pushingbox.com"; //pushingbox API server
const String devid = "v9606769D2CC718F"; //device ID on Pushingbox for our Scenario

//***WIFI SETTINGS***
const char* MY_SSID = "PSU";//"PSU"; //does not currently seem to want to connect to PSU network, can connect at home fine
const char* MY_PWD =  "";   //""; //wifi password
const int secured = 0; //change to 1 if connecting to secured WIFI
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

//Define analog input and values for EC4-500-CO sensor:
int COSensorValue = 0;
int CO = 0;

//make a SHT31 object
Adafruit_SHT31 sht31 = Adafruit_SHT31();

int status = WL_IDLE_STATUS; //global to avoid passing

/**************/
/**Functions**/
/*************/
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void Con2wifi(){
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network: ");
    Serial.println(MY_SSID);
    
    if(secured)
      status = WiFi.begin(MY_SSID, MY_PWD); //connect to secured wifi
    else
      status = WiFi.begin(MY_SSID); //connect to open wifi
   delay(10000);  // wait 10 seconds for connection:
  }
}
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  starttime = millis();
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  Serial.println("serial conection initialized");
  pinMode(PPMPIN, INPUT); // PPM
  pinMode(OZONEPIN, INPUT); // Ozone
  pinMode(IRQ_PIN, INPUT_PULLUP); // interrupts from RTC
  pinMode(COPIN, INPUT); // CO
  // attempt to connect to Wifi network:
  //should ifdef its out
  Con2wifi(); //connect to the wifi
  //endifdef
  Serial.println("Connected to wifi");
  printWifiStatus();
  //end ifdef
  if (!sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
  }
  Serial.println("starting SD");
  if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("started SD");
  //else Serial.println("found SHT31");

  //set up RTC
  rtc.begin(RTC_CS);
  //rtc.autoTime(); //set time to compiler time. will be a little off but VERY close
  rtc.update(); //needs to be done to set alarms
  //rtc.enableAlarmInterrupt();
  //rtc.setAlarm1(30); //alarm1 alert when seconds hits 30
  //rtc.setAlarm1(rtc.minute() + 5); //alarm1 triggered when minute increments by 5
}


//this code is only used by K-30 C02 sensor.
//later i will make a different version of the code for with/without CO2
//because it takes a up a lot of space and is ugly.
//alternatively ifdef it out
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

void printdata(float Temp, float Hum, float PPM, float Ozone, float CO) {
  rtc.update();
  String htime = String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second());
  String hdate = String(rtc.month()) + "/" + String(rtc.date()) + "/" + String(rtc.year()) + " - ";
  Serial.println(hdate + htime);
  Serial.print("Temperature: ");
  Serial.print(Temp);
  Serial.print(" *C\n");
  Serial.print("Humidity: ");
  Serial.print(Hum);
  Serial.print("%\n");
  Serial.print("PPM:  ");
  Serial.print(PPM);
  Serial.print("\n");
  Serial.print("Ozone Concentration: ");
  Serial.print(Ozone);
  Serial.print("\n");
  Serial.print("CO Concentration: ");
  Serial.print(CO);
  Serial.println("\n");
}

//function to write data to SD card.
int logdata(float Temp, float Hum, float PPM, float Ozone, float CO) {

  //string that gets written to SD card
  String WriteMe = "";

  //open file
  File W_File = SD.open(FILENAME, FILE_WRITE); // write file, if that wasnt clear
  if (W_File) {

    //prepping data for ease of appending to WriteMe
    float data[5]; //should really use a #define instead of hardcoded 5. somthing like NUM_SENSORS
    //hard-coded because i dont know how to make dynamic arrays nicely
    data[0] = Temp;
    data[1] = Hum;
    data[2] = PPM;
    data[3] = Ozone;
    data[4] = CO;

    //WriteMe += hdate + htime; // timestamp at beginning
    //get time
    rtc.update();
    String htime = String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second());
    String hdate = String(rtc.month()) + "/" + String(rtc.date()) + "/" + String(rtc.year());

    if (firstrun) {
      firstrun = 0; //the global
      W_File.println("Beginning of this data block");
      W_File.println("Date,Time,Temperature(Celcius),Humidity(%),Particles(pcs/.01 cubic feet),Ozone(PPB),CO(PPM)");
      W_File.println();
    }
    W_File.print(hdate + ","); //make nice CSV's
    W_File.print(htime + ",");//make nice CSV's
    for (int i = 0; i < 5; ++i) {
      W_File.print(String(data[i]));
      //WriteMe += String(data[i]);
      if (i != 4) //no delimeter after final data item
        W_File.print(",");
    }
    W_File.println();
    W_File.close();
  }


  else {
    Serial.println("error opening file");
    return 1; //
  }
  return 0; //no return = good
}

//function for sending data to cloud. right now it uses google sheets
//but it may get changed to azure since sheets isnt very elegant
int postdata(float Temp, float Hum, float PPM, float Ozone, float CO) {
  if (status != WL_CONNECTED){
    Con2wifi();
    printWifiStatus();
  }
  Serial.println("\nSending Data to Server..."); 
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

      // HTTP 1.1 provides a persistent connection, allowing batched requests
      // or pipelined to an output buffer
      client.println(" HTTP/1.1"); 
      client.print("Host: ");
      client.println(WEBSITE);
      client.println("User-Agent: MKR1000/1.0");
      //for MKR1000, unlike esp8266, do not close connection
      client.println();
      client.stop();
      Serial.println("\nData Sent"); 
    }
   
    else{
      Serial.println("ERROR CONNECTING TO SERVER");
      WiFi.disconnect();
      status = WL_IDLE_STATUS;
    }
}

//main
void loop() {
  //noInterrupts(); //turn interrupts off
  //wake up

  // Wait between measurements.
  //can probably be replaced by waiting for IRQ from RTC
  delay(SAMPLE_RESOLUTION);
  //Data Assignments
  //Use float for higher resolution

  //read temp
  float Temp = sht31.readTemperature();
  //read humidity
  float Hum = sht31.readHumidity();
  //get ppm information
  float PPM = getppm();//concentration;
  //read O3
  float Ozone = analogRead(OZONEPIN); //Read value from ozone pin
  //Ozone = 385 - (Ozone / 2); //ozone w/ a calibration curve
  //read CO
  float CO = analogRead(COPIN);

  printdata(Temp, Hum, PPM, Ozone, CO);
  logdata(Temp, Hum, PPM, Ozone, CO);
  postdata(Temp,Hum,PPM,Ozone,CO);
  //go back to sleep
  //interrupts(); //turn interrupts back on
}
