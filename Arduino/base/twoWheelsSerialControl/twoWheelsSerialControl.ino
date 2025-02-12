/*   Hoverboard_Serial_Test
 *   Controls the speed, brake, and direction of a single hoverboard motor
 *   via commands sent through the serial port.
 *   Measures the speed of a hoverboard motor asynchronously
 *   using a custom ReadSpeed function.  Uses the SC speed pulse output of the
 *   RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall.
 *   Outputs the speed data to the serial port.
 *     
 *   created 2021
 *   Mad-EE  (Mad Electrical Engineer)
 *   www.mad-ee.com
 *   
 *   This example code is in the public domain.
 *   
 *   Platform:  Arduino UNO
 */

// Constants
const unsigned long SPEED_TIMEOUT = 40000;       // Time used to determine wheel is not spinning
const unsigned int UPDATE_TIME = 500;             // Time used to output serial data
const unsigned int BUFFER_SIZE = 16;              // Serial receive buffer size
const double BAUD_RATE = 115200;                  // Serial port baud rate

// Pin Declarations

const int R_PIN_DIR = 2;      // Motor direction signal
const int R_PIN_PWM = 3;      // PWM motor speed control
const int R_PIN_SPEED = 4;   // SC Speed Pulse Output from RioRand board
const int R_PIN_BRAKE = 5;    // Motor brake signal (active low)

const int L_PIN_DIR = 8;      // Motor direction signal
const int L_PIN_PWM = 9;      // PWM motor speed control
const int L_PIN_SPEED = 10;   // SC Speed Pulse Output from RioRand board
const int L_PIN_BRAKE = 11;    // Motor brake signal (active low)

// Variables used in ReadFromSerial function
String _command = "";       // Command received in Serial read command
int _data = 0;              // Data received in Serial read command

// Variables used in ReadSpeed function
double l_period_uS;               // Frequency of the signal on the speed pin
double l_sp_period = SPEED_TIMEOUT; // set point
double l_p_err; // error periodo
double l_p_err_old = 0; // error periodo antiguo
double l_p_err_sum = 0; // suma del error
int l_pwm = 0; // final PWM controlled

double r_period_uS;               // Frequency of the signal on the speed pin
double r_sp_period = SPEED_TIMEOUT; // set point
double r_p_err; // error periodo
double r_p_err_old = 0; // error periodo antiguo
double r_p_err_sum = 0; // suma del error
int r_pwm = 0; // final PWM controlled

double l_period_uS_mean;
const int l_numReadings = 64;
double l_p[l_numReadings];
double l_total_p = 0.0;
int l_readIndex = 0;

double r_period_uS_mean;
const int r_numReadings = 64;
double r_p[r_numReadings];
double r_total_p = 0.0;
int r_readIndex = 0;

// This is ran only once at startup
void setup()  {
    // Set pin directions
    pinMode(L_PIN_SPEED, INPUT);
    pinMode(L_PIN_PWM, OUTPUT);
    pinMode(L_PIN_BRAKE, OUTPUT);
    pinMode(L_PIN_DIR, OUTPUT);

    pinMode(R_PIN_SPEED, INPUT);
    pinMode(R_PIN_PWM, OUTPUT);
    pinMode(R_PIN_BRAKE, OUTPUT);
    pinMode(R_PIN_DIR, OUTPUT);
    
    // Set initial pin states
    digitalWrite(L_PIN_BRAKE, false);
    digitalWrite(L_PIN_DIR, false);
    analogWrite(L_PIN_PWM, 0);

    digitalWrite(R_PIN_BRAKE, false);
    digitalWrite(R_PIN_DIR, true);
    analogWrite(R_PIN_PWM, 0);

    // Initialize serial port
    Serial.begin(BAUD_RATE);
    Serial.println("---- Program Started ----");
    delay(2000);
}

// This is the main program loop that runs repeatedly
void loop() {
    // Read serial data and set dataReceived to true if command is ready to be processed
    bool dataReceived = ReadFromSerial();

    // Process the received command if available
    if (dataReceived == true)
        ProcessCommand(_command, _data);

    // Read the speed from input pin (sets _rpm and _mph)
    ReadSpeed();

    // Control loop
    leftControlLoop();
    rightControlLoop();
    smoothValue( l_pwm, r_pwm );

    // Outputs the speed data to the serial port 
    WriteToSerial(); 
}

int rightControlLoop() {
  double Kp = 0.9;//.03;
  double Kd = 1.1E-6;//.03;
  double Ki = 0.09;//0001;
  double a = 7.92E-7;
  double b = -1.96E-2;
  double c = 150;
  
  int pwm0 = a * r_sp_period * r_sp_period + b * r_sp_period + c;
  pwm0 = constrain(pwm0, 0, 255);
  r_p_err = (r_period_uS_mean - r_sp_period) / r_sp_period;
  r_pwm = pwm0 + Kp * r_p_err + Ki * (r_p_err_sum) + Kd*(r_p_err_old - r_p_err);
  r_p_err_sum += r_p_err;
  r_p_err_old = r_p_err;
  r_pwm = constrain(r_pwm, 0, 255);
//  analogWrite( R_PIN_PWM, r_pwm );
  return r_pwm;
}

int leftControlLoop() {
  double Kp = 0.9;//.03;
  double Kd = 1.1E-6;//.03;
  double Ki = 0.09;//0001;
  double a = 7.92E-7;
  double b = -1.96E-2;
  double c = 150;

  int pwm0 = a * l_sp_period * l_sp_period + b * l_sp_period + c;
  pwm0 = constrain(pwm0, 0, 255);
  l_p_err = (l_period_uS_mean - l_sp_period) / l_sp_period;
  l_pwm = pwm0 + Kp * l_p_err + Ki * (l_p_err_sum) + Kd*(l_p_err_old - l_p_err);
  l_p_err_sum += l_p_err;
  l_p_err_old = l_p_err;
  l_pwm = constrain(l_pwm, 0, 255);
//  analogWrite( L_PIN_PWM, l_pwm );
  return l_pwm;
}

void smoothValue( int l_p, int r_p) {
  double a = 0.95;
  int l_p_smoothed = 0;
  int l_p_old = 0;
  int r_p_smoothed = 0;
  int r_p_old = 0;
  do {
    l_p_smoothed = l_p * (1-a) + l_p_old * a;
    r_p_smoothed = r_p * (1-a) + r_p_old * a;
    l_p_old = l_p_smoothed;
    r_p_old = r_p_smoothed;
  } while( ((l_p_smoothed - l_p) < a ) & ((r_p_smoothed - r_p) < a ) );
}

bool ReadFromSerial() {    
    // Local variables   
    static String cmdBuffer;        // Stores the received command
    static String dataBuffer;       // Stores the received data
    static bool isCommand = true;   // Flag to store received bytes in command or data buffer
    byte recByte;                   // Byte received from the serial port
    
    if (Serial.available() == false) return false;
    
    recByte = Serial.read();
    
    if (recByte == '\r') {
        cmdBuffer.toUpperCase();
        _command = cmdBuffer;
        _data = dataBuffer.toInt();
      
        Serial.print("Received: "); 
        Serial.print(_command); 
        Serial.print(",");
        Serial.println(_data);
      
        cmdBuffer = "";
        dataBuffer = "";
        isCommand = true;
  
        return true;
    }
    
    if ((char)recByte == ',') {
        isCommand = false;  // Next byte will be a data byte
        return false;
    }

    if (isCommand) cmdBuffer += (char)recByte;
    else dataBuffer += (char)recByte;

    return false;
}

void ProcessCommand(String command, int data) {  
    if (command == "R_RPM") {
      Serial.print("Setting speed right:  ");
      Serial.println(data);
      r_sp_period = data;
    }
    if (command == "L_RPM") {
      Serial.print("Setting speed left:  ");
      Serial.println(data);
      l_sp_period = data;
    }
    if (command == "RPM") {
      Serial.print("Setting speed right left:  ");
      Serial.println(data);
      l_sp_period = data;
      r_sp_period = data;
    }
        
    // Process BRAKE command
    if (command == "BRAKE") {
      Serial.print("Setting brake:  ");
      Serial.println(data);
      digitalWrite(R_PIN_BRAKE, data);
      digitalWrite(L_PIN_BRAKE, data);
    }

    // Process DIR command
    if (command == "R_DIR") {
      Serial.print("Setting direction right:  ");
      Serial.println(data);
      digitalWrite(R_PIN_DIR, data);
    }
    if (command == "L_DIR") {
      Serial.print("Setting direction left:  ");
      Serial.println(data);
      digitalWrite(L_PIN_DIR, data);
    }
}

// Reads the speed from the input pin and calculates RPM and MPH
// Monitors the state of the input pin and measures the time (µs) between pin transitions
void ReadSpeed() {
    static bool leftLastState = false;    // Saves the last state of the speed pin
    static bool rightLastState = false;    // Saves the last state of the speed pin
    static unsigned long left_last_uS;     // The time (µs) when the speed pin changes
    static unsigned long right_last_uS;     // The time (µs) when the speed pin changes
    static unsigned long left_timeout_uS;  // Timer used to determine the wheel is not spinning
    static unsigned long right_timeout_uS;  // Timer used to determine the wheel is not spinning

    // Read the current state of the input pin
    bool left_state = digitalRead(L_PIN_SPEED);
    bool right_state = digitalRead(R_PIN_SPEED);

    // Check if the pin has changed state
    if (left_state != leftLastState) {
      // Calculate how long has passed since last transition
      unsigned long current_uS = micros();
      unsigned long elapsed_uS = current_uS - left_last_uS;

      // Calculate the frequency of the input signal
      l_period_uS = elapsed_uS * 2.0;
      left_last_uS = current_uS;
      left_timeout_uS = left_last_uS + SPEED_TIMEOUT;
      leftLastState = left_state;
    } else if (micros() > left_timeout_uS) {
        l_period_uS = SPEED_TIMEOUT;
        left_last_uS = micros();
    }
    // Check if the pin has changed state
    if (right_state != rightLastState) {
      // Calculate how long has passed since last transition
      unsigned long current_uS = micros();
      unsigned long elapsed_uS = current_uS - right_last_uS;

      // Calculate the frequency of the input signal
      r_period_uS = elapsed_uS * 2.0;
      right_last_uS = current_uS;
      right_timeout_uS = right_last_uS + SPEED_TIMEOUT;
      rightLastState = right_state;
    } else if (micros() > right_timeout_uS) {
        r_period_uS = SPEED_TIMEOUT;
        right_last_uS = micros();
    }
      l_total_p = l_total_p - l_p[l_readIndex];
      l_p[l_readIndex] = l_period_uS;
      l_total_p = l_total_p + l_p[l_readIndex];
      l_readIndex ++;
      if(l_readIndex >= l_numReadings) l_readIndex = 0;
      l_period_uS_mean = l_total_p / l_numReadings;

      r_total_p = r_total_p - r_p[r_readIndex];
      r_p[r_readIndex] = r_period_uS;
      r_total_p = r_total_p + r_p[r_readIndex];
      r_readIndex ++;
      if(r_readIndex >= r_numReadings) r_readIndex = 0;
      r_period_uS_mean = r_total_p / r_numReadings;
}

// Writes the RPM and MPH to the serial port at a set interval
void WriteToSerial() {
    static unsigned long updateTime;
    
    if (millis() > updateTime) {
        Serial.print((String)"R_Period:" + r_period_uS + " ");
        Serial.print((String)"R_PeriodMean:" + r_period_uS_mean + " ");
        Serial.print((String)"R_SetPoint:" + r_sp_period + " ");
        Serial.print((String)"R_Perr:" + r_p_err + " ");
        Serial.print((String)"R_PWM:" + r_pwm + " ");

        // Calculate next update time
        Serial.print((String)"L_Period:" + l_period_uS + " ");
        Serial.print((String)"L_PeriodMean:" + l_period_uS_mean + " ");
        Serial.print((String)"L_SetPoint:" + l_sp_period + " ");
        Serial.print((String)"L_Perr:" + l_p_err + " ");
        Serial.println((String)"L_PWM:" + l_pwm + " ");

        updateTime = millis() + UPDATE_TIME;
    }
}