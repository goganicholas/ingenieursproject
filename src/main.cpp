//WaterqualiTDS
#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
 
//waterflowsensor
volatile double waterFlow;
 
//temperatuursensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4 // Data wire is conntec to the Arduino digital pin 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
//soilmoisture
const int AirValue = 610; //you need to change this value that you had recorded in t
const int WaterValue = 70; //you need to change this value that you had recorded in th
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;

//lichtsensor
#include <BH1750.h>
#include <Wire.h>
BH1750 lightMeter;
 
void setup() {
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
 
void WaterflowSensor()
{
 
}
 
void pulse()   //measure the quantity of square wave
{
  waterFlow += 1.0 / 5880.0;
}
 
void Temperatuursensor(){ 
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 
  delay(500);
}
 
void GrondVochtigheid() {
  soilMoistureValue = analogRead(A15); //put Sensor insert into soil
}
 
void Lichtsensor() {
  //float lux = lightMeter.readLightLevel();
}
 
void loop(){
  WaterKwaliteit();
  WaterflowSensor();
  Temperatuursensor();
  GrondVochtigheid();
  Lichtsensor();
 
  //print kwaliteit
  Serial.print("TDS Value:");
  Serial.print(tdsValue,0);
  Serial.println("ppm");
 
  //print waterflow
  Serial.print("waterFlow:");
  Serial.print(waterFlow);
  Serial.println("   L");
 
  //print temperatuur
  float Temperatuur = sensors.getTempCByIndex(0);
  Serial.print("Celsius temperature: ");
  Serial.println(Temperatuur); 
  //Serial.print(" - Fahrenheit temperature: ");
  //Serial.println(sensors.getTempFByIndex(0));
 
  //print grondvochtigheid
  if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals))
    {
      Serial.print("Very Wet");
    }
  else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
    {
      Serial.print("Wet");
    }
  else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals))
    {
      Serial.println("Dry");
    }
 
  //print licht
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
 
  delay(1000);
 
}