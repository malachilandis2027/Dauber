#include <MBED_RPi_Pico_TimerInterrupt.h>
#include <MBED_RPi_Pico_TimerInterrupt.hpp>
#include <MBED_RPi_Pico_ISR_Timer.h>
#include <MBED_RPi_Pico_ISR_Timer.hpp>



/*
This code runs the stepper and spindle motors
A laptop should be connected to the control cabinet, then a serial monitor sends commands, such as:
- SP100 - Sets the spindle to 100% power
- SP0 - Sets the spindle to 0% power (off)
- FR0.5D10.0 - Sets the stepper feedrate to 0.5mm/s for a duration of 10s
- FR0 - Sets the stepper feedrate to 0mm/s (off)
- off/Off/OFF - turns everything off
*/

/*
Quick note about the pins on the Arduino RP2040 Connect.
The pinout document has labels of DX and AX, where X is a number. It also has GPIOXXX labels.
Turns out that the Arduino MBED board package (that shows up when you look for this board in the IDE),
wants pins to be labeled pX, where X is the number from the GPIO label on the pinout image.
*/


// Defining pins for spindle control
#define SP_POWER_FWD p5 // D10 // Pin for controlling PWM level of the spindle motor in the forward direction
#define SP_POWER_REV p20 // D8 // Pin for controlling PWM level of the spindle motor in the reverse direction
#define SP_ENABLE_FWD p21 // D9 // Pin for enabling forward rotation of the spindle motor
#define SP_ENABLE_REV p19 // D7 // Pin for enabling reverse rotation of the spindle motor

// Defining pins for stepper control
#define STP_PUL p25 // D2 // Step pin
#define STP_DIR p15 // D3 // Direction pin
#define STP_EN p16 // D4 // Enable pin

// Tool parameters
#define FEED_WHEEL_DIAMETER 25 // Diameter of feed wheel in [mm]
#define STEPS_PER_REV 5370 // Steps per revolution of the feed wheel
#define PI 3.141592
#define TIMER_INTERVAL_US 2000 // How often the stepper ISR runs in unsigned long [us]

unsigned long countsToStep = 100; // Number of times the ISR should run before it takes a step
unsigned long countsDuration = 0; // How many times the ISR should update before it doesn't step regardless
unsigned long countsSinceStepped = 0; // How many times the ISR has updated since it last called for a step
unsigned long countsSinceStarted = 0; // How many times the ISR has updated since the command started

bool lastDirection = true; // Last direction the motor rotated, true -> forward, false -> reverse
bool currentDirection = false; // Current direction the motor is rotating, true -> forward, false -> reverse

// ----------------------------- Helpers ----------------------------

// Takes a step with the stepper motor
void takeStep(bool direction) {
  // Inputs:
  //   - direction - true > forward, false > reverse
  if (direction != lastDirection) { // First check whether the stepper is changing direction
    if (direction) {
      digitalWrite(STP_DIR,HIGH); // Set the direction pin high (forward)
    }
    else {
      digitalWrite(STP_DIR,LOW); // Set the direction pin low (reverse)
    }
    lastDirection = direction; // Update the last direction
    delayMicroseconds(6); // Delay as required by the drive after setting the direction pin
  }
  digitalWrite(STP_PUL,HIGH); // Set the pulse pin high
  delayMicroseconds(3); // Delay as required by the drive
  digitalWrite(STP_PUL,LOW); // Set the pulse pin low
  delayMicroseconds(3); // Delay as required by the drive
}

// Turn the stepper driver on
void enableStepper() {
  digitalWrite(STP_EN,LOW); // Set the enable pin low to enable the drive
  delayMicroseconds(250000); // Delay needed before stepping occurs
}

// Turn the stepper driver off
void disableStepper() {
  digitalWrite(STP_EN,HIGH); // Set the enable pin high to disable the drive
}

// Use the feedrate and duration to set the counts per step and total counts
void setStepperFeedrateDuration(float feedrate, float duration) {
  if (feedrate < 0) {
    currentDirection = true;
  }
  else {
    currentDirection = false;
  }

  // The feedrate is in [mm/s] and needs to be converted to number of ISR calls in between steps
  countsToStep = (FEED_WHEEL_DIAMETER * PI * 1000000) / (STEPS_PER_REV * feedrate * TIMER_INTERVAL_US); // Updates per step

  // The duration is in [s] and needs to be converted to number of ISR calls
  countsDuration = 1000000 * duration / TIMER_INTERVAL_US; // Counts total for command
}

// Set the spindle power using a setting between -100 to 100 (corresponding to full reverse and full forward, respectively)
void setSpindlePower(int power) {
  int convertedPower = map(power, -100, 100, -255, 255);
  if (power > 0) { // Positive power, so spindle should be moving forward
    digitalWrite(SP_ENABLE_REV,LOW); // Disable the reverse direction
    digitalWrite(SP_ENABLE_FWD,HIGH); // Enable the forward direction
    analogWrite(SP_POWER_REV,0); // Disable reverse PWM
    analogWrite(SP_POWER_FWD,convertedPower); // Set correct forward PWM
  }
  else if (power < 0) { // Negative power, so spindle should be moving backward
    digitalWrite(SP_ENABLE_FWD,LOW); // Disable the forward direction
    digitalWrite(SP_ENABLE_REV,HIGH); // Enable the reverse direction
    analogWrite(SP_POWER_FWD,0); // Disable forward PWM
    analogWrite(SP_POWER_REV,-convertedPower); // Set correct reverse PWM
  }
  else { // 0 power, spindle should be off
    digitalWrite(SP_ENABLE_FWD,LOW); // Disable the forward direction
    digitalWrite(SP_ENABLE_REV,LOW); // Disable the reverse direction
    analogWrite(SP_POWER_FWD,0); // Disable forward PWM
    analogWrite(SP_POWER_REV,0); // Disable reverse PWM
  }
}

// ISR to be run at the interupt frequency, this steps when necessary
void stepperUpdateISR(unsigned int alarm_num) {
  countsSinceStarted = countsSinceStarted + 1;
  // if (countsSinceStepped >= countsToStep && countsSinceStarted < countsDuration) { // Check to see if the ISR has run enough to warrant a step being taken AND that it should still be stepping
  //   countsSinceStepped = 0; // Reset this counter
  //   if (currentDirection != lastDirection) { // First check whether the stepper is changing direction
  //     if (currentDirection) {
  //       digitalWrite(STP_DIR,HIGH); // Set the direction pin high (forward)
  //     }
  //     else {
  //       digitalWrite(STP_DIR,LOW); // Set the direction pin low (reverse)
  //     }
  //   lastDirection = currentDirection; // Update the last direction
  //   delayMicroseconds(6); // Delay as required by the drive after setting the direction pin
  //   }
  //   digitalWrite(STP_PUL,HIGH); // Set the pulse pin high
  //   delayMicroseconds(3); // Delay as required by the drive
  //   digitalWrite(STP_PUL,LOW); // Set the pulse pin low
  //   delayMicroseconds(3); // Delay as required by the drive
  // }
  // else {
  //   countsSinceStepped++; // Increment the stepping counter
  //   countsSinceStarted++; // Increment the duration counter
  // }
}

// ------------------------------ Setup -----------------------------

MBED_RPI_PICO_Timer ITimer(1);

void setup() {
  Serial.begin(115200); // Start serial communication with laptop
  delay(5000);
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_US, stepperUpdateISR)) {
    Serial.println("Starting stepper ISR");
  } 
  else {
    Serial.println("Couldn't start stepper ISR");
  }
}

// ---------------------------- Main Loop  --------------------------

void loop() {
  Serial.println(countsSinceStarted);
  delay(200);
  if (Serial.available()) { // Check to see if there is something to read
    String userInput = String(Serial.readStringUntil('\n'));
    // Begin by checking the first two characters

    String firstTwo = userInput.substring(0,2);
    String firstThree = userInput.substring(0,3);

    // Spindle control command processing
    if (firstTwo == "SP") { // The characters "SP" indicate spindle control command
      int power = userInput.substring(2).toInt(); // Get the power level as an integer by reading the remainder of the string
      if (power > 100 || power < -100) { // Check the bounds on the power level
        Serial.println("Error in specifying spindle power level");
      }
      else { // Set the spindle PWM power level
        Serial.print("Setting spindle power to ");
        Serial.println(power);
        setSpindlePower(power);
      }
    }

    // Feedrate control command processing
    else if (firstTwo == "FR") { // The characters "FR" indicate feedrate control command
      int durationIndex = userInput.indexOf('D'); // Find the first instance of "D", after which the duration is specified
      if (durationIndex == -1) {
        Serial.println("Error, no \"D\" character found in feedrate control command");
      }
      else {
        enableStepper();
        float feedrate = userInput.substring(2,durationIndex).toFloat(); // Get the feedrate as the float between "FR" and "D"
        float duration = userInput.substring(durationIndex+1).toFloat(); // Get the duration as the remaining float after "D"

        Serial.print("Setting stepper feedrate to ");
        Serial.print(feedrate);
        Serial.print(" [mm/s] and the duration to ");
        Serial.print(duration);
        Serial.println(" [s]");
        countsSinceStarted = 0;
        setStepperFeedrateDuration(feedrate, duration);
      }
    }

    else if (firstThree == "off" || firstThree == "Off" || firstThree == "OFF") { // Off commands
      Serial.println("Turning spindle and stepper off");
      setSpindlePower(0);
      setStepperFeedrateDuration(0.0,0.0);
      disableStepper();
    }

    else {
      Serial.println("Input error");
    }

  }
}
