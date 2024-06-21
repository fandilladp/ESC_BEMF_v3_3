/* Title: ELECTRONOOBS open source electronic speed controller.
 * Date: 02/04/2019
 * UPDATE: 06/07/2019
 * Version: 3.3
 * Author: http://electronoobs.com
 * Tutorial link: https://www.electronoobs.com/eng_arduino_tut91.php
 * Schematic link: https://www.electronoobs.com/eng_arduino_tut91_sch1.php
 * PCB gerbers: https://www.electronoobs.com/eng_arduino_tut91_gerbers1.php
 * This is a sensorless ESC based on Arduino with the ATmega328 chip. It uses
 * BEMF with the internal comparator of the ATmega328 to detect the rotor position.
 * The speed control is made by a PWM signal. Feel free to change it and improve
 * it however you want
 * Subscribe: http://youtube.com/c/ELECTRONOOBS
 */

#include "EEPROMAnything.h"  // This is used to store more than just one byte to the EEPROM
#include "Beeps.h"
#include "Variables.h"

void setup() {
    pinMode(buzzer, OUTPUT);

    // This will only run once after you upload the code
    if (EEPROM.read(1) != 1) {
        EEPROM_writeAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);
        EEPROM_writeAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);
        EEPROM.write(1, 1);
    }

    Serial.begin(9600);
    EEPROM.write(1, 0);
    DDRD |= B00011100;   // Configure pins 2, 3, and 4 as outputs
    PORTD = B00000000;   // Pins 0 to 7 set to LOW
    DDRB |= B00001110;   // Configure pins 9, 10, and 11 as outputs
    PORTB &= B00000000;  // Pins 9, 10, and 11 to LOW

    // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
    TCCR1A = 0;
    TCCR1B = 0x01;
    // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
    TCCR2A = 0;
    TCCR2B = 0x01;
    // Analog comparator setting
    ACSR = 0x10;  // Disable and clear (flag bit) analog comparator interrupt

    // Set D8 (PWM in) to trigger interrupt
    PCICR |= (1 << PCIE0);    // Enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0);  // Set pin D8 to trigger an interrupt on state change.

    delay(200);

    // Power on mode select
    if (PWM_INPUT > PWM_IN_MIN + 115) {  // Enter config mode to set the PWM range
        ESC_MODE_ON = false;  // Motor rotation is OFF till the config mode is done
        while (!PWM_RANGE_SET) {
            currentMillis = millis();
            if (currentMillis - previousMillis >= 500) {
                OCR1A = beeping_PWM_VALUE;
                previousMillis += 500;
                TCCR2A = 0;  // OC2A - D11 normal port.
                TCCR1A = 0;  // OC1A and OC1B normal port
                beep_1KHZ(100);
            }

            if (PWM_INPUT > MAX_PWM_TO_STORE) {
                MAX_PWM_TO_STORE = PWM_INPUT;
            }

            if (PWM_INPUT < 1200) {
                if (pwm_set_counter > 1000) {
                    MIN_PWM_TO_STORE = PWM_INPUT;
                    EEPROM_writeAnything(PWM_IN_MIN_ADRESS, MIN_PWM_TO_STORE);
                    EEPROM_writeAnything(PWM_IN_MAX_ADRESS, MAX_PWM_TO_STORE);
                    ESC_MODE_ON = true;
                    PWM_RANGE_SET = true;
                    TCCR2A = 0;  // OC2A - D11 normal port.
                    TCCR1A = 0;  // OC1A and OC1B normal port
                    beep_1KHZ(400);
                    delay(200);

                    TCCR2A = 0;  // OC2A - D11 normal port.
                    TCCR1A = 0;  // OC1A and OC1B normal port
                }
            }
        }
    }

    ESC_MODE_ON = true;
}

void loop() {
    if (ESC_MODE_ON) {
        if (PWM_INPUT > PWM_IN_MIN + 115 && PWM_INPUT < PWM_IN_MAX) {
            // Add motor control logic here
            MOTOR_SPINNING = true;
        } else {
            MOTOR_SPINNING = false;
        }
    }
}

ISR(PCINT0_vect) {
    // Add interrupt service routine logic here
}
