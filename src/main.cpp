#include <Arduino.h>
#include <Wire.h> 
// #include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000

// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}

#define outPin 8        // Defines pin number to which the sensor is connected

// dht DHT;                // Creates a DHT object


#define PIR A3
#define temp A2
#define moisture A6
#define sound A7
#define delay_MS 1000

unsigned long time,time2,currentmillis;
//int x=0,y=0;
String msg,data;



// LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SoftwareSerial mySerial(4,3); 
bool pir_state,Sound_Sate,Moisture_State;
float bodytemperature;
// float humidity, temperature, temperatureF;

bool CheckMotion() {
  bool val = digitalRead(PIR);
  return val;
}
bool CheckSound() {
  bool Cry = 0;
  for(int i = 0; i <10; i++)
  {
    if (analogRead(A7) < 500) {
      Cry = 1;
    }
  }
  return Cry;
}

String prepare_msg(float j,bool k, bool q, bool r) {
  String check = ("Baby is moving.");
  String alert = ("Baby is awake and crying !");
  String diaper,SMS;

  if (k == 1 ) {
    diaper = ("Diaper is Wet !");
   }
   else if(k == 0) {
    diaper = ("Diaper is dry");
   }
  if(q==1 && r==0 ) {
    SMS = (check + "   Temperature: " + String(j) + "C  " + diaper); 
  }
  else if (r == 1 && q==0) {
    SMS = (alert + "   Temperature: " + String(j) + "C  " + diaper); 
  }
  else if(r == 0 && q==0 && k ==1){
    SMS = (diaper + "   Temperature: " + String(j) + "C  ");
  }
  else {
    SMS = "Null";
  }
  
  return SMS;
}

float Readtemperature() {
  int reading = analogRead(temp);
  float voltage = reading * (5.0 / 1024.0);
  float tempC = ((voltage * 100)) ;
  return tempC;
  
}

bool Check_Moisture() {
  bool m = 0;
  int x = analogRead(moisture);
  if (x < 800) m = 1;
  return m;
}

void updateSerial()
{
  // delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

void sendSMS(String pq) {
  mySerial.begin(9600);
  Serial.println("PQ====================> ");
  Serial.print(pq);
  Serial.println("Initializing....");
  // delay(1000);
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  // mySerial.println("AT+CMGS=\"+8801758805632\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  //updateSerial();
  updateSerial();
  mySerial.print(pq);
  updateSerial();
  mySerial.write(26);      
}




void setup() {
  Serial.begin(9600);
  //sim_Initialize();
  Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

	
  // lcd.init();                      // initialize the lcd 
  // lcd.backlight();
  pinMode(PIR,INPUT);
  pinMode(temp,INPUT);
  pinMode(moisture,INPUT);
  pinMode(sound,INPUT);
  // lcd.setCursor(0,0);
  // lcd.print("Baby Monitoring System");
  // delay(2000);
  time = millis();
  time2 = millis();

  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Temp:");
  // lcd.setCursor(10,0);
  // lcd.print((char)223);
  // lcd.setCursor(11,0);
  // lcd.print("f");

  // Configure sensor to use 7.6mA for LED drive
	pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback routine
  pox.setOnBeatDetectedCallback(onBeatDetected);


}
// void lcdprint(){
//   // lcd.clear();
//   // lcd.setCursor(0,0);
//   // lcd.print("Temp:");
//   // lcd.setCursor(11,0);
//   // lcd.print((char)223);
//   // lcd.setCursor(12,0);
//   // lcd.print("c");
//   // lcd.setCursor(0,1);
//   // lcd.print("BPM:");
//   // lcd.setCursor(6,1);
//   // lcd.print("SpO2:");
// }

void loop() {
  // Serial.println(millis());
  pox.update();

  // DHT.read11(outPin);

	// temperature = DHT.temperature;        // Read temperature
	// humidity  = DHT.humidity;           // Read humidity
  // temperatureF = ((temperature*9.0)/5.0+32.0);

  

  // Grab the updated heart rate and SpO2 levels
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      Serial.print("Heart rate:");
      Serial.print(pox.getHeartRate());
      Serial.print("bpm / SpO2:");
      Serial.print(pox.getSpO2());
      Serial.println("%");

      tsLastReport = millis();
  }


  pir_state = CheckMotion();
  bodytemperature = Readtemperature();
  Sound_Sate = CheckSound();
  Moisture_State = Check_Moisture();

    // if(millis() - currentmillis >= 1000ul){
    // // lcd.setCursor(5,0);
    // // lcd.print(bodytemperature,2);
    // // lcd.setCursor(4,1);
    // // lcd.print(BPM);
    // // lcd.setCursor(11,1);
    // // lcd.print(SP02);
    // }
    currentmillis = millis();

  if ((pir_state == 1 || Sound_Sate == 1 || Moisture_State ==1 ) && (( millis() - time2) > 10000) ) 
  {
    sendSMS(prepare_msg(bodytemperature,Moisture_State,pir_state,Sound_Sate));
    Serial.print("Sending msg");
    time2 = millis();
  }
  

  if (millis() - time > delay_MS)
  {
    Serial.print(bodytemperature); Serial.print("\xC2\xB0"); Serial.print("f"); 
    Serial.println();
    Serial.print(analogRead(moisture));
    Serial.print(":::::::::::::::::::");
    Serial.println();
    // lcd.scrollDisplayLeft();
    // lcd.print("Baby Monitoring System");
    time = millis();
  }
  

  if (pir_state == 1)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    // lcd.setCursor(1,1);
    // lcd.print("Baby woken up !!");
    pir_state = 1;
  }
  else
  {
    digitalWrite(LED_BUILTIN,LOW);
    // delay(5000);
    // lcd.clear();
    // lcdprint();
  }
  updateSerial();

  // Read from the sensor
  
}