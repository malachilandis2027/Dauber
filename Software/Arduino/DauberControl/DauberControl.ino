#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>

/*
This code runs the stepper and spindle motors
A laptop should be connected to the control cabinet, then a serial monitor sends commands, such as:
- SP100 - Sets the spindle to 100% power
- SP0 - Sets the spindle to 0% power (off)
- FR0.5D10.0 - Sets the stepper feedrate to 0.5mm/s for a duration of 10s
- FR0 - Sets the stepper feedrate to 0mm/s (off)
- off/Off/OFF - turns everything off
*/

// Defining pins for spindle control
#define SP_POWER_FWD D2 // Pin for controlling PWM level of the spindle motor in the forward direction
#define SP_POWER_REV D3 // Pin for controlling PWM level of the spindle motor in the reverse direction
#define SP_ENABLE_FWD D4 // Pin for enabling forward rotation of the spindle motor
#define SP_ENABLE_REV D5 // Pin for enabling reverse rotation of the spindle motor

// Defining pins for stepper control
#define STP_STEP D6 // Step pin
#define STP_DIR D7 // Direction pin
#define STP_EN D8 // Enable pin

// Tool parameters
#define FEED_WHEEL_DIAMETER 25 // Diameter of feed wheel in [mm]
#define STEPS_PER_REV 5370 // Steps per revolution of the feed wheel

#define TIMER_FREQ_HZ 5000.0 // Update rate for stepper
int updatesToStep = 100; // Number of times the ISR should run before 


// This function runs at the interupt frequency and steps when necessary
void stepperUpdateISR() {

}

// Sets the spindle power using a setting between -100 to 100
void setSpindlePower(int power) {
  int convertedPower = map(power, -100, 100, -255, 255)
  if (power > 0) { // Positive power, so spindle should be moving forward
    analogWrite(SP_POWER_REV,0); // Disable reverse PWM first
    analogWrite(SP_POWER_FWD,convertedPower); // Set correct forward PWM
  }
  else { // power <= 0
    analogWrite(SP_POWER_FWD,0); // Disable forward PWM first
    analogWrite(SP_POWER_REV,-convertedPower); // Set correct reverse PWM
  }
}

void setup() {
  Serial.begin(115200); // Start serial communication with laptop
}

void loop() {
  if (Serial.available()) { // Check to see if there is something to read
    String userInput = String(Serial.readStringUntil('\n'));
    // Begin by checking the first two characters
    if (userInput.substring(0,2) == "SP") { // Spindle control
      int power = userInput.substring(2).toInt(); // Get the power level as an integer
      if (power > 100 || power < 0) { // Check the bounds on the power level
        Serial.println("Error in specifying spindle power level");
      }
      else { // Set the spindle PWM power level
        setSpindlePower(power);
      }
    }
    else if (userInput.substring(0,2) == "FR") { // Feedrate control

    }
    else {
      Serial.println("Input error");
    }
    // Read the serial value as a feedrate in [mm/s]

  }
}
