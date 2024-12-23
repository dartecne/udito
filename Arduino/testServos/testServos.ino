#include <Servo.h>

#define PAN_PIN 45
#define TILT_PIN  9

Servo pan;  // create servo object to control a servo
Servo tilt;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int min = 20;
int max = 40;
int tau = 60;
void setup() {
  pan.attach(PAN_PIN);  // attaches the servo on pin 9 to the servo object
  tilt.attach(TILT_PIN);  // attaches the servo on pin 9 to the servo object
  tilt.write(0);
  delay(1000);
  tilt.write(20);
  delay(1000);
  tilt.write(0);
  delay(1000);
//  while(1);
}

void loop() {
  test_tilt();
}
void test_pan() {
  for (pos = 0; pos <= 40; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pan.write(pos);              // tell servo to go to position in variable 'pos'
    delay(tau);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 40; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    pan.write(pos);              // tell servo to go to position in variable 'pos'
    delay(tau);                       // waits 15 ms for the servo to reach the position
  }
}

void test_tilt() {
  for (pos = min; pos <= max; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    tilt.write(pos);              // tell servo to go to position in variable 'pos'
    delay(tau);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = max; pos >= min; pos -= 1) { // goes from 180 degrees to 0 degrees
    tilt.write(pos);              // tell servo to go to position in variable 'pos'
    delay(tau);                       // waits 15 ms for the servo to reach the position
  }

}