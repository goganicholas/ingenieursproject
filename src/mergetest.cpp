#include <Arduino.h>


//LCDscherm
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
 
//Lampen
int led1 = 13; //oranje
int led2 = 12; //half - half
int led3 = 11; //wit
 
//Pomp
const int pomp = 10;
 
//WaterqualiTDS
#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

//testmergeleds
                int greenLedPin = 3; // Pin connected to the green LED
                int yellowLedPin = 4; // Pin connected to the yellow LED
                int redLedPin = 5; // Pin connected to the red LED




//waterflowsensor
volatile double waterFlow;
 
//temperatuursensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 25 // Data wire is conntec to the Arduino digital pin 25
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float Temperatuur;
 
//soilmoisture
const int AirValue = 610; //gemeten waarde lucht
const int WaterValue = 70; //gemeten waarde water
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue;
 
//lichtsensor
#include <BH1750.h>
#include <Wire.h>
BH1750 lightMeter;
int lux;
 
void setup() {
  pinMode(pomp, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
 
  lcd.init();
  lcd.clear();
 
  Serial.begin(115200);
  pinMode(TdsSensorPin,INPUT);
  waterFlow = 0;
  attachInterrupt(0, pulse, RISING);  //DIGITAL Pin 2: Interrupt 0
  sensors.begin();
 
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(0x23);
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  // For Wemos / Lolin D1 Mini Pro and the Ambient Light shield use
  // Wire.begin(D2, D1);
 
  lightMeter.begin();
 
  Serial.println(F("BH1750 Test begin"));

//testmerge
                    pinMode(greenLedPin, OUTPUT); // Set the green LED pin as output
                    pinMode(yellowLedPin, OUTPUT); // Set the yellow LED pin as output
                    pinMode(redLedPin, OUTPUT); // Set the red LED pin as output
 
}
 
void WaterKwaliteit() {
 
  static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
    }   
  static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
    } 
                                                              // TEST KLEURCODE LEDS WATERQUALI
    if (tdsValue > 15 && tdsValue <= 50) {
        digitalWrite(greenLedPin, HIGH); // Turn on the green LED
        digitalWrite(yellowLedPin, LOW); // Turn off the yellow LED
        digitalWrite(redLedPin, LOW); // Turn off the red LED
        delay(500); // Delay for 500 milliseconds
    } else if (tdsValue > 50 && tdsValue <= 100) {
        digitalWrite(greenLedPin, LOW); // Turn off the green LED
        digitalWrite(yellowLedPin, HIGH); // Turn on the yellow LED
        digitalWrite(redLedPin, LOW); // Turn off the red LED
        delay(1000); // Delay for 1000 milliseconds
    } else if (tdsValue > 100) {
        digitalWrite(greenLedPin, LOW); // Turn off the green LED
        digitalWrite(yellowLedPin, LOW); // Turn off the yellow LED
        digitalWrite(redLedPin, HIGH); // Turn on the red LED
        delay(2000); // Delay for 2000 milliseconds
    } else {
        digitalWrite(greenLedPin, LOW); // Turn off the green LED
        digitalWrite(yellowLedPin, LOW); // Turn off the yellow LED
        digitalWrite(redLedPin, LOW); // Turn off the red LED
        delay(200); // Delay for 200 milliseconds
    }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
 
/*
void WaterflowSensor()
{
  mss gebruiken we die nog idk
}
*/
 
void pulse()   //measure the quantity of square wave
{
  waterFlow += 1.0 / 5880.0;
}
 
void Temperatuursensor(){ 
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 
  Temperatuur = sensors.getTempCByIndex(0);
 
}
 
void GrondVochtigheid() {
  soilMoistureValue = analogRead(A15); //put Sensor insert into soil
}
 
void Lichtsensor() {
  lux = lightMeter.readLightLevel();
}
 
void LCD()
{
  lcd.backlight();
}
 
//waarden moeten nog aangepast worden - 4000 is veel te hoog
void Lampen() { 
  if (lux < 4000){
    int brightness1 = 70;
    int brightness2 = 50;
    analogWrite( led2, brightness2);
    analogWrite( led1, brightness1 );} else if(lux < 5000){
      int brightness2 = 50;
      int brightness3 = 30;
      analogWrite( led3, brightness3);
      analogWrite( led2, brightness2);} else if(lux < 6000){
        int brightness3 = 30;
        analogWrite( led3, brightness3);} else{
          int brightness1 = 0;
          int brightness2 = 0;
          int brightness3 = 0;}
}
 
void Pomp() {
  if (soilMoistureValue > 430) {      //pomp aan laten spirngen wanneer grond droog is
    digitalWrite(pomp, HIGH);
  }
  if (soilMoistureValue < 250) {           //pomp af laten springen wanneer grond nat is
    digitalWrite(pomp,LOW);
  }
}
 
  void loop(){
 
  WaterKwaliteit();
  // WaterflowSensor();          Deze functie gebruiken we niet, de volledige code hiervan is gecomment (LN123-127)
  Temperatuursensor();
  GrondVochtigheid();
  Lichtsensor();
  LCD();
  Lampen();
  Pomp();
 
  //print kwaliteit
  Serial.print("TDS Value:");
  Serial.print(tdsValue,0);
  Serial.println("ppm");
 
  //print waterflow
  Serial.print("waterFlow:");
  Serial.print(waterFlow);
  Serial.println("   L");
 
  //print temperatuur
  Serial.print("Celsius temperature: ");
  Serial.println(Temperatuur);
 
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(Temperatuur);
  lcd.print(" C");
 
  //print grondvochtigheid
  lcd.setCursor(0, 1);
  if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals)) //70 < waarde < 250
    {
      Serial.println("Vocht: Very Wet, Waarde: ");
      Serial.println(soilMoistureValue);
      lcd.print("Vocht: Very Wet");
    }
  else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals)) //250 < waarde < 430
    {
      Serial.println("Vocht: Wet, Waarde: ");
      Serial.println(soilMoistureValue);
      lcd.print("Vocht: Wet");
    }
  else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals)) //430 < waarde < 610
    {
      Serial.print("Vocht: Dry, Waarde: ");
      Serial.println(soilMoistureValue);
      lcd.print("Vocht: Dry");
    }
 
  //print licht
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
 
  delay(500);
 
}