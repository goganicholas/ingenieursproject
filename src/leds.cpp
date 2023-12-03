#include <Arduino.h>


int tdsValue;
int greenLedPin = 13; // Pin connected to the green LED
int yellowLedPin = 12; // Pin connected to the yellow LED
int redLedPin = 11; // Pin connected to the red LED

void setup() {
    pinMode(greenLedPin, OUTPUT); // Set the green LED pin as output
    pinMode(yellowLedPin, OUTPUT); // Set the yellow LED pin as output
    pinMode(redLedPin, OUTPUT); // Set the red LED pin as output
}

void waterqualiteitKLEUR() {
    tdsValue = analogRead(A0); // Read the sensor value from analog pin A0

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
