
#include <Arduino.h>

//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
#define USE_LIGHTWEIGHT_SERVO_LIBRARY   // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
#define ENABLE_MIN_AND_MAX_CONSTRAINTS     // Activating this enables constraint checking. Requires 4 bytes RAM per servo and 36 bytes program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define MAX_EASING_SERVOS 2
//#define TRACE
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.
//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.

#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"

#define START_DEGREE_VALUE  0 // The degree value written to the servo at time of attach.
#define SERVO1_PIN  9
int servo1_pin = 9;
ServoEasing Servo1;

int v =60;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
    Serial.println(F("Operate servo 1 from -90 to + 90 degree by using attachWithTrim()"));
    Servo1.attachWithTrim(servo1_pin, 90, START_DEGREE_VALUE - 90, DEFAULT_MICROSECONDS_FOR_0_DEGREE,
    DEFAULT_MICROSECONDS_FOR_180_DEGREE);
//    Servo1.setMinMaxConstraint(-50, +50);
    setSpeedForAllServos(30);
    Servo1.setEasingType(EASE_CUBIC_IN_OUT);
    Servo1.printStatic(&Serial);
    delay(1000);
}

void loop() {
    Serial.println(F("Move back"));
//    setSpeedForAllServos(30);
    Servo1.setEaseTo(-40.0f, v); // Use x.y with trailing f (to specify a floating point constant) to avoid compiler errors.
    synchronizeAllServosStartAndWaitForAllServosToStop();
    while (ServoEasing::areInterruptsActive()) {
        // Here you can insert your own code
      blinkLED();
    }
//    delay(2000);

    Serial.println(F("Move fwd"));
    Servo1.setEaseTo(10.0f, v);
    synchronizeAllServosStartAndWaitForAllServosToStop();

    while (ServoEasing::areInterruptsActive()) {
        // Here you can insert your own code
        blinkLED();
    }

    /*
     * Another method to specify moves
     * Use the ServoEasingNextPositionArray and then call the appropriate function
     * /
    ServoEasing::ServoEasingNextPositionArray[0] = 0.0f;
    ServoEasing::ServoEasingNextPositionArray[1] = 30.0f;
    setEaseToForAllServosSynchronizeAndStartInterrupt(30); // Set speed and start interrupt here, we check the end with areInterruptsActive()

    // Call yield for the ESP boards must be handled in areInterruptsActive()
    while (ServoEasing::areInterruptsActive()) {
        ; // Here you can insert your own code
    }
*/
 //   delay(2000);
    
}

void blinkLED() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
