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

int min_servo = 1200;
int max_servo = 1800;

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
  int tau = 4;
  for( int p = min_servo; p < max_servo; p++ ){
    pot1 = p;
    pot2 = p;
    pot3 = map(p, min_servo, max_servo, max_servo, min_servo);

    Serial.print(pot1);
    Serial.print(" , ");
    Serial.print(pot2);
    Serial.print(" , ");
    Serial.println(pot3);

    servo1.writeMicroseconds(pot1);      // pan
    servo2.writeMicroseconds(pot2);      // neck left
    servo3.writeMicroseconds(pot3);      // neck right
    delay(tau);
  }    
  for( int p = max_servo; p >= min_servo; p-- ){
    pot1 = p;
    pot2 = p;
    pot3 = map(p, min_servo, max_servo, max_servo, min_servo);

    Serial.print(pot1);
    Serial.print(" , ");
    Serial.print(pot2);
    Serial.print(" , ");
    Serial.println(pot3);

    servo1.writeMicroseconds(pot1);      // pan
    servo2.writeMicroseconds(pot2);      // neck left
    servo3.writeMicroseconds(pot3);      // neck right
    delay(tau);
  }    
  delay(500);
} // end if main loop



