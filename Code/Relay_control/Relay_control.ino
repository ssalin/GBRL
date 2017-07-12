//CONSTANTS
int POSITION_TIME = 5000; // 5 seconds
int SAMPLE_TIME = 1000; // 1 second
//PIN INFO
int relay1 = 0;
int relay2 = 1;
int relay3 = 2;
int relay4 = 3;
int relay5 = 4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(relay1, OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT); 
  pinMode(relay5,OUTPUT);
}

void loop() {
  for (int i=0; i<5; ++i){
    digitalWrite (i, HIGH);
    digitalWrite (relay5, HIGH);
    delay (POSITION_TIME);  
    digitalWrite (i, LOW);
    digitalWrite (relay5, LOW);
    delay (SAMPLE_TIME);
  }
/*  
  digitalWrite(relay1, HIGH); //move to position 1
  digitalWrite(relay5, HIGH); // release pressure
  delay (POSITION_TIME);
  
  digitalWrite(relay1, LOW); //TURN OFF RELAY 1
  digitalWrite(relay5, LOW); //TURN OFF RELEASE
  delay(SAMPLE_TIME);

  digitalWrite(relay2, HIGH); //move to position 2
  digitalWrite(relay5, HIGH); // release pressure
  delay(POSITION_TIME);

  digitalWrite(relay2, LOW); //TURN OFF RELAY 2
  digitalWrite(relay5, LOW); //TURN OFF PRESSURE RELIEF
  delay(SAMPLE_TIME);

  digitalWrite(relay3, HIGH); //move to position 3
  digitalWrite(relay5, HIGH); // release pressure
  delay (POSITION_TIME);
  
  digitalWrite(relay3, LOW); //TURN OFF RELAY 3
  digitalWrite(relay5, LOW); //TURN OFF PRESSURE RELIEF
  delay (SAMPLE_TIME);

  digitalWrite(relay4, HIGH);
  digitalWrite(relay5, HIGH);
  delay(POSITION_TIME)

  digitalWrite(relay4, LOW);
  digitalWrite(relay5, LOW):
  delay(SAMPLE_TIME);

  
*/
}
