
#include <Adafruit_NeoPixel.h>
#include "eyes_data.h"

#define LED_PIN    5
#define LED_COUNT 128

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t r = strip.Color(255,   0,   60);
static uint32_t b = strip.Color( 60,   0,   255);
static uint32_t color = strip.Color( 0,   0,   255);
int o = 0;
int n_options = 9;
int tau = 900;

void setup() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
// colorWipe(b, 20);  
  clear(0);
  clear(1);
  show_love(tau);
//  while(1);
  o = int(random(n_options));
}

void loop(){
  o = int(random(n_options));
//  o = int(random(5,8));
//  o = 5;
  tau = int(random(20, 300));
  switch(o){
    case 0:
      celebration(tau);
      break;
    case 1:
      blink(tau);
      break;
    case 2:
      happy();
      break;
    case 3:
      sad(tau);
      delay(2*tau);
      break;
    case 4:
      angry(tau);
      delay(2*tau);
      break;
    case 5:
      show_logo_1(3*tau);
      delay(2*tau);
      show_logo_1(tau);
      break;
    case 6:
      show_logo_2(3*tau);
      delay(2*tau);
      show_logo_2(tau);
      break;
    case 7:
      show_logo_3(3*tau);
      delay(2*tau);
      show_logo_3(tau);
      break;
    case 8:
      show_love(tau);
      break;
  }
  delay(10*tau);
  blink_line(tau);
}

void show_love(int tau) {
  color = r;
  showShape(heart_1, 0);
  showShape(heart_1, 1);
  delay( tau );
  showShape(heart_2, 0);
  showShape(heart_2, 1);
  delay( tau );
  showShape(heart_3, 0);
  showShape(heart_3, 1);
  delay( tau );
  showShape(heart_2, 0);
  showShape(heart_2, 1);
  delay( tau );
  showShape(heart_1, 0);
  showShape(heart_1, 1);
  delay( tau );
  showShape(heart_2, 0);
  showShape(heart_2, 1);
  delay( tau );
  showShape(heart_3, 0);
  showShape(heart_3, 1);
  delay( tau );
  color = b;
}

void show_logo_1(int tau) {
  showShape(letter_u, 0);
  showShape(letter_u, 1);
  delay( tau );
  showShape(letter_d, 0);
  showShape(letter_d, 1);
  delay( tau );
  showShape(letter_i, 0);
  showShape(letter_i, 1);
  delay( tau );
  showShape(letter_t, 0);
  showShape(letter_t, 1);
  delay( tau );
}

void show_logo_2(int tau) {
  showShape(letter_u, 0);
  showShape(letter_d, 1);
  delay( tau );
  showShape(letter_d, 0);
  showShape(letter_i, 1);
  delay( tau );
  showShape(letter_i, 0);
  showShape(letter_t, 1);
  delay( tau );
//  showShape(letter_t, 0);
//  showShape(letter_t, 1);
//  delay( tau );
}

void show_logo_3(int tau) {
  showShape(letter_u, 0);
  showShape(letter_d, 1);
  delay( tau );
  showShape(letter_i, 0);
  showShape(letter_t, 1);
  delay( tau );
}

void laugh(){
  showShape(laugh_left, 0);
  showShape(laugh_right, 1);
}

void sad( int tau ){
  showShape(sad_1_left, 0);
  showShape(sad_1_right, 1);
  delay(tau);
  showShape(sad_2_left, 0);
  showShape(sad_2_right, 1);
  delay(tau);
  showShape(sad_1_left, 0);
  showShape(sad_1_right, 1);
  delay(tau);
  showShape(sad_2_left, 0);
  showShape(sad_2_right, 1);
  delay(tau);
}

void happy(){
  showShape(happy_1, 0);
  showShape(happy_1, 1);
}

void angry(int tau){
  showShape(angry_1_left, 0);
  showShape(angry_1_right, 1);
  delay(tau);
  showShape(angry_2_left, 0);
  showShape(angry_2_right, 1);
  delay(tau);
  showShape(angry_1_left, 0);
  showShape(angry_1_right, 1);
  delay(tau);
  showShape(angry_2_left, 0);
  showShape(angry_2_right, 1);
  delay(tau);
}

void celebration(int tau){
  showShape( neutral_1_medium, 0 );
  showShape( neutral_1_medium, 1 );
  delay(2*tau);
  showShape( neutral_1_big, 0 );
  showShape( neutral_1_small, 1 );
  delay(5*tau);
  showShape( neutral_1_medium, 0 );
  showShape( neutral_1_medium, 1 );
  delay(2*tau);
  showShape( neutral_1_small, 0 );
  showShape( neutral_1_big , 1 );
  delay(5*tau);
  showShape( neutral_1_medium, 0 );
  showShape( neutral_1_medium, 1 );
  delay(2*tau);
}

void blink_line( int tau ) {
  showShape( t_1_line, 0 );
  showShape( t_1_line, 1 );
  delay(tau);
  showShape( t_2_line, 0 );
  showShape( t_2_line, 1 );
  delay(tau);
  showShape( t_3_line, 0 );
  showShape( t_3_line, 1 );
  delay(tau);
  showShape( t_4_line, 0 );
  showShape( t_4_line, 1 );
  delay(tau);
  showShape( t_4_line, 0 );
  showShape( t_4_line, 1 );
  delay(tau);
  showShape( t_3_line, 0 );
  showShape( t_3_line, 1 );
  delay(tau);
  showShape( t_2_line, 0 );
  showShape( t_2_line, 1 );
  delay(tau);
  showShape( t_1_line, 0 );
  showShape( t_1_line, 1 );  
  delay(tau);
}

void blink( int tau ){
  showShape( t_1, 0 );
  showShape( t_1, 1 );
  delay(tau);
  showShape( t_2, 0 );
  showShape( t_2, 1 );
  delay(tau);
  showShape( t_3, 0 );
  showShape( t_3, 1 );
  delay(tau);
  showShape( t_4, 0 );
  showShape( t_4, 1 );
  delay(tau);
  showShape( t_4, 0 );
  showShape( t_4, 1 );
  delay(tau);
  showShape( t_3, 0 );
  showShape( t_3, 1 );
  delay(tau);
  showShape( t_2, 0 );
  showShape( t_2, 1 );
  delay(tau);
  showShape( t_1, 0 );
  showShape( t_1, 1 );
}

void wink( int tau, int eyeID ) {
  showShape( t_1, eyeID );
  delay(tau);
  showShape( t_2, eyeID );
  delay(tau);
  showShape( t_3, eyeID );
  delay(tau);
  showShape( t_4, eyeID );
  delay(tau);
  showShape( t_4, eyeID );
  delay(tau);
  showShape( t_3, eyeID );
  delay(tau);
  showShape( t_2, eyeID );
  delay(tau);
  showShape( t_1, eyeID );
}

void showShape(int (&shape)[N][N], int eyeID) {
  clear( eyeID );
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j ++) {
      int k = toVector(j,i, eyeID);
      if(shape[i][j]  == 1) strip.setPixelColor(k, color);         //  Set pixel's color (in RAM)
    }
  }
  strip.show();                          //  Update strip to match
}

void test() {
  colorWipe(strip.Color(255,   0,   0), 50); // Red
  colorWipe(strip.Color(  0, 255,   0), 50); // Green
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  rainbow(10);             // Flowing rainbow cycle along the whole strip
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
}

void clear(int eyeID) {
  for(int i = eyeID*N*N; i < eyeID*N*N + N*N; i++) strip.setPixelColor(i,strip.Color(0, 0, 0)) ;
  strip.show();
}

int toVector(int i, int j, int eyeID ) {
  return j*N+i + eyeID*N*N;
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
