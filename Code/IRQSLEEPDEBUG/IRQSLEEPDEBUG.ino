//test program to debug irq functionality with the MKR1000.
//program goes to sleep and waits for an interrupt from the clock. 
//when the interrupt comes the device wakes up and an LED is lit.
//LED is connected to pin A4

#include <SparkFunDS3234RTC.h> //rtc lib
#include <SPI.h> //needed for RTC

#define RTC_CS 6    //chip select for RTC
#define RTC_IRQ A2  // A2 connected to SQW of RT

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  pinMode(A4, OUTPUT);
  //pinMode(RTC_IRQ, INPUT_PULLUP); // interrupts from RTC
  rtc.begin(RTC_CS);
  Serial.println("Boutta delay for reprogramming");
  delay(20000); //20 seconds or so, time to uploadd a new program
  Serial.println("past the delay");
  EIC->WAKEUP.reg |= (1 << RTC_IRQ); //SHOULD let interrupts wakeup from sleep
  attachInterrupt(RTC_IRQ,wakeup, LOW);
  rtc.enableAlarmInterrupt();
   
  //turn on clock to allow for falling/change/rising IRQ triggers 
  // Set the XOSC32K to run in standby
  //SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
  // Configure EIC to use GCLK1 which uses XOSC32K 
  // This has to be done after the first call to attachInterrupt()
  //GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | 
  //                    GCLK_CLKCTRL_GEN_GCLK1 | 
  //                    GCLK_CLKCTRL_CLKEN;
}

void LEDON() {
  digitalWrite(A4, HIGH);
}
void LEDOFF(){
  digitalWrite(A4,LOW);
}

//irq callback function
void wakeup(){
  Serial.println("Entered IRQ callback");
  rtc.update();
  if(rtc.alarm1()){
    Serial.println("ALARM 1 TRIGGERED");
    LEDON();//turn on LED  
  } 
  //if(rtc.alarm2())
  //  LEDOFF();
}

void slep(){
    rtc.update(); //fix no wakeup?
    rtc.setAlarm1(rtc.minute() + 1); //alarm1 triggered when minute increments by 1
    Serial.println("Boutta turn IRQs on");
    //interrupts(); //turn interrupts on
    Serial.println("boutta go to sleep");
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; //turn on deep sleep SAMD board specific.
    __DSB(); //finish mem accesses
    Serial.println("pre __WFI");
    __WFI(); //wait for interrupt
    Serial.println("Post __WFI");
    delay(5000);
    LEDOFF();
}

void loop() {
  slep();
}
