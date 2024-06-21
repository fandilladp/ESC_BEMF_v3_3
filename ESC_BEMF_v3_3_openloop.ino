#include "Variables.h"

void setup() {
    pinMode(buzzer, OUTPUT);
    Serial.begin(9600);

    // Configure motor control pins as outputs
    DDRD |= B00011100;   // Configure pins 2, 3, and 4 as outputs
    PORTD = B00000000;   // Pins 0 to 7 set to LOW
    DDRB |= B00001110;   // Configure pins 9, 10, and 11 as outputs
    PORTB &= B00000000;  // Pins 9, 10, and 11 set to LOW

    // Set up PWM for motor control
    TCCR1A = 0;    // Clear Timer1 settings
    TCCR1B = 0x01; // Set Timer1 prescaler to 1
    TCCR2A = 0;    // Clear Timer2 settings
    TCCR2B = 0x01; // Set Timer2 prescaler to 1

    // Initial delay to ensure proper setup
    delay(200);
}

void loop() {
    // Open-loop control: directly set motor control pins
    PORTB |= (1 << PB1);  // Set PB1 (pin 9) to HIGH
    delay(1000);          // Motor runs for 1 second

    PORTB &= ~(1 << PB1); // Set PB1 (pin 9) to LOW
    PORTB |= (1 << PB2);  // Set PB2 (pin 10) to HIGH
    delay(1000);          // Motor runs for 1 second

    PORTB &= ~(1 << PB2); // Set PB2 (pin 10) to LOW
    PORTB |= (1 << PB3);  // Set PB3 (pin 11) to HIGH
    delay(1000);          // Motor runs for 1 second

    PORTB &= ~(1 << PB3); // Set PB3 (pin 11) to LOW
}

ISR(PCINT0_vect) {
    // No interrupt handling in open-loop test
}
