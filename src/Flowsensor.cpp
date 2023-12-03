#include <Arduino.h>

const int flowSensorPin = 2; // Afhankelijk van op welke pin we de sensor aansluiten
volatile int totalLiters = 0; // Variabele om de totale liters water op te slaan

const float pulsesPerLiter = 1000; // Aantal pulsen per liter water

void pulseCounter()
{
    totalLiters += 1 / pulsesPerLiter; // Verhoog het totale aantal liters met de hoeveelheid water die wordt gedetecteerd
}

void setup()
{
    Serial.begin(9600); // Initialiseer de seriële communicatie
    pinMode(flowSensorPin, INPUT); // Stel de pin van de watersensor in als input
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, RISING); // Koppel de interrupt aan de pin van de watersensor
/*

Wanneer er een stijgende signaalflank (van laag naar hoog) wordt gedetecteerd op deze pin
wordt de functie pulseCounter geactiveerd. Deze functie telt het aantal pulsen,
wat overeenkomt met de hoeveelheid water die door de sensor stroomt.
*/


}

void loop()
{
    // Print het totale aantal liters water in de seriële monitor
    Serial.print("Totaal Liters: ");
    Serial.println(totalLiters);
    delay(1000); // Wacht 1 seconde
}
