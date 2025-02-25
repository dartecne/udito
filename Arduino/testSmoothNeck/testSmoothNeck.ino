#include <Servo.h>
#define PAN_PIN 2
#define LEFT_PIN  3
#define RIGHT_PIN 4

int pot1 = 512;
int pot2 = 512;
int pot3 = 512;

float pot1Scaled;
float pot2Scaled;
float pot3Scaled;

float pot1Smoothed = 512;
float pot2Smoothed = 512;
float pot3Smoothed = 512;

float pot1SmoothedPrev = 512;
float pot2SmoothedPrev = 512;
float pot3SmoothedPrev = 512;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 5;        // time constant for timer

int stepFlag = 0;
long previousStepMillis = 0;

Servo servo1;
Servo servo2;
Servo servo3;

int min_servo = 200;
int max_servo = 1000;

void setup() {

  Serial.begin(115200);

  servo1.attach( PAN_PIN ); // pan
  servo2.attach( LEFT_PIN ); //
  servo3.attach( RIGHT_PIN );
  //[1000, 2000] // [CCW, CW], 1500 - middle
  //[700, 2300]
  servo1.writeMicroseconds(1200);
  servo2.writeMicroseconds(1650);
  servo3.writeMicroseconds(1650);

}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  // start 5ms timed loop  
      previousMillis = currentMillis;

      // step sequencer

      if (stepFlag == 0 && currentMillis - previousStepMillis > 500) {
        pot1 = 512;
        stepFlag = 1;           
        previousStepMillis = currentMillis;
      }

      else if (stepFlag == 1 && currentMillis - previousStepMillis > 1000) {
//        pot1 = 1024;
        pot2 = max_servo;
        pot3 = min_servo;
        stepFlag = 2;           
        previousStepMillis = currentMillis;
      }

      else if (stepFlag == 2 && currentMillis - previousStepMillis > 1500) {
//        pot1 = 0;
        pot2 = min_servo;
        pot3 = max_servo;
        stepFlag = 3;           
        previousStepMillis = currentMillis;
      }

      if (stepFlag == 3 && currentMillis - previousStepMillis > 1500) {
        pot2 = 512;
        pot3 = 512;
        stepFlag = 4;           
        previousStepMillis = currentMillis;
      }

      if (stepFlag == 4 && currentMillis - previousStepMillis > 1500) {
        pot2 = min_servo;
        pot3 = min_servo;
        stepFlag = 5;           
        previousStepMillis = currentMillis;
      }
      if (stepFlag == 5 && currentMillis - previousStepMillis > 1500) {
        pot2 = max_servo;
        pot3 = max_servo;
        stepFlag = 1;           
        previousStepMillis = currentMillis;
      }

      // end of step sequencer

      
    
      // scale all pots for the servo microseconds range
    
      pot1Scaled = ((pot1 - 512) * -1.6) + 1200;
      pot2Scaled = (pot2 - 512) + 1500;
      pot3Scaled = (pot3 - 512) + 1500;

//      pot4Scaled = constrain(pot4Scaled,900,1500);
    
      // smooth pots
      
      pot1Smoothed = (pot1Scaled * 0.01) + (pot1SmoothedPrev * 0.99);
      pot2Smoothed = (pot2Scaled * 0.01) + (pot2SmoothedPrev * 0.99);
      pot3Smoothed = (pot3Scaled * 0.01) + (pot3SmoothedPrev * 0.99);
    
      // bookmark previous values
    
      pot1SmoothedPrev = pot1Smoothed;
      pot2SmoothedPrev = pot2Smoothed;
      pot3SmoothedPrev = pot3Smoothed;
  
      Serial.print(pot1Smoothed);
      Serial.print(" , ");
      Serial.print(pot2Smoothed);
      Serial.print(" , ");
      Serial.println(pot3Smoothed);
      // write servos

      servo1.writeMicroseconds(pot1Smoothed);                     // neck rotate
      servo2.writeMicroseconds(pot2Smoothed);      // neck left
      servo3.writeMicroseconds(pot3Smoothed);      // neck right
    
  } // end of timed loop


} // end if main loop