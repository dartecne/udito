/*   
 Main program for controlling Base Wheels
 TODO: tune PID controller
*/

// Constants
#define SPEED_TIMEOUT 80000       // Time used to determine wheel is not spinning
#define UPDATE_TIME 100            // Time used to output serial data
#define BUFFER_SIZE 16              // Serial receive buffer size
#define BAUD_RATE 115200                  // Serial port baud rate

// Pin Declarations

#define R_PIN_DIR 2      // Motor direction signal
#define R_PIN_PWM 3      // PWM motor speed control
#define R_PIN_SPEED 4   // SC Speed Pulse Output from RioRand board
#define R_PIN_BRAKE 5    // Motor brake signal (true frena)

#define L_PIN_DIR 7      // Motor direction signal
#define L_PIN_PWM 9      // PWM motor speed control
#define L_PIN_SPEED 10   // SC Speed Pulse Output from RioRand board
#define L_PIN_BRAKE 11    // Motor brake signal (active wheel low)

// Variables used in ReadFromSerial function
String _command = "";       // Command received in Serial read command
int _data = 0;              // Data received in Serial read command

bool l_status = 0; // break ON
bool r_status = 0; // break ON

#define l_min_freq 0 // stopped
#define l_max_freq 6250 // max velocity 16000 us period ( 1/ T ) * 1E6 * 1E2 (añade 2 cifras significativas al Hz)
#define r_min_freq 0 // stopped
#define r_max_freq 6250 // max velocity 16000 us period

// Variables used in ReadSpeed function
double l_freq;               // Frequency of the signal on the speed pin. in centiHerz (100cHz = 1Hz)
double l_sp = l_min_freq; // set point
double l_err; // error 
double l_err_old = 0; // error periodo antiguo
double l_err_sum = 0; // suma del error
double l_pwm = 0; // final PWM controlled
unsigned long l_now, l_before;
double l_ellapsed_time;

double r_freq;               // Frequency of the signal on the speed pin
double r_sp = r_min_freq; // set point
double r_err; // error 
double r_err_old = 0; // error periodo antiguo
double r_err_sum = 0; // suma del error
double r_pwm = 0; // final PWM controlled
unsigned long r_now, r_before;
double r_ellapsed_time;


double l_freq_mean;
const int l_numReadings = 100;

double l[l_numReadings];
double l_total = 0.0;
int l_readIndex = 0;

double r_freq_mean;

const int r_numReadings = 100;
double r[r_numReadings];
double r_total = 0.0;
int r_readIndex = 0;
bool noLoop = false;

double Kp = 0.1;//0.02;
double Kd = 0;//0.7;//0.05;//0.1;//0.01;//30;//0.5;//2;
double Ki = 0;//1E-7;
 

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
    digitalWrite(L_PIN_BRAKE, 1);
    digitalWrite(L_PIN_DIR, 1);
    analogWrite(L_PIN_PWM, 0);

    digitalWrite(R_PIN_BRAKE, 1);
    digitalWrite(R_PIN_DIR, 0);  //   1- bwd, 0-fwd
    analogWrite(R_PIN_PWM, 0);
    delay(2000);

    // Initialize serial port
    Serial.begin(BAUD_RATE);
    Serial.println("ACK");
    Serial.print("l_freq - l_freq_mean - l_sp - l_error - l_error_sum - l_pwm");
    Serial.println(" = r_freq - r_freq_mean - r_sp - r_error - r_error_sum - r_pwm");

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
  if(l_status) leftControlLoop();
  else l_pwm = 0;
  if(r_status) rightControlLoop();
  else r_pwm = 0;
  if( noLoop == false ) { // if PWM, no control Loop is made
    analogWrite( L_PIN_PWM, constrain(round(l_pwm), 0, 255)); 
    analogWrite( R_PIN_PWM, constrain(round(r_pwm), 0, 255));
  }
    // Outputs the speed data to the serial port 
    WriteToSerial(); 
//   logData();
}

void rightControlLoop() {
  r_now = millis();
  r_ellapsed_time = (double)( r_now - r_before);
  
  r_err = (r_freq_mean - r_sp);
    
  r_err_sum += r_err * r_ellapsed_time;
  r_pwm = r_pwm + Kp * r_err + Ki * (r_err_sum) + Kd*(r_err_old - r_err) / (r_ellapsed_time + 1);
  
  r_err_old = r_err;
  r_before = r_now;
  
  if(r_pwm > 255 ) r_pwm = 255.0;
  if(r_pwm < 0 ) r_pwm = 0.0;
}

void leftControlLoop() {
  l_now = millis();
  l_ellapsed_time = (double)( l_now - l_before);
  
  l_err = (l_freq_mean - l_sp);
    
  l_err_sum += l_err * l_ellapsed_time;
  l_pwm = l_pwm + Kp * l_err + Ki * (l_err_sum) + Kd*(l_err_old - l_err) / (l_ellapsed_time + 1);
  
  l_err_old = l_err;
  l_before = l_now;
  
  if(l_pwm > 255 ) l_pwm = 255.0;
  if(l_pwm < 0 ) l_pwm = 0.0;
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
      
//        Serial.print("Received: "); 
//        Serial.print(_command); 
//        Serial.print(",");
  //      Serial.println(_data);
      
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
    if (command == "PWM") {
      digitalWrite(L_PIN_BRAKE, false);
      digitalWrite(R_PIN_BRAKE, false);
      r_status = 1;
      l_status = 1;
      analogWrite(L_PIN_PWM, data);
      analogWrite(R_PIN_PWM, data);
      noLoop = true;
    } 
    if (command == "R_V") {
      Serial.print("Setting right:  ");
      Serial.println(data);
      r_sp = map(data, 0, 100, r_min_freq, r_max_freq);
      digitalWrite(R_PIN_BRAKE, false);
      r_status = 1;
      noLoop = false;
    }
    if (command == "L_V") {
      Serial.print("Setting left:  ");
      Serial.println(data);
      l_sp = map(data, 0, 100, l_min_freq, l_max_freq);
      digitalWrite(L_PIN_BRAKE, false);
      l_status = 1;
      noLoop = false;
    }
    if (command == "V") {
//      Serial.print("Setting velocity right left:  ");
//      Serial.println(data);
      r_sp = map(data, 0, 100, r_min_freq, r_max_freq);
      l_sp = map(data, 0, 100, l_min_freq, l_max_freq);
      digitalWrite(L_PIN_BRAKE, false);
      digitalWrite(R_PIN_BRAKE, false);
      r_status = 1;
      l_status = 1;
      noLoop = false;
    }
        
    // Process BRAKE command
    if (command == "BRAKE") {
      Serial.print("Setting brake:  ");
      Serial.println(data);
      digitalWrite(R_PIN_BRAKE, data);
      digitalWrite(L_PIN_BRAKE, data);
      r_status = !data;
      l_status = !data;
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
    if (command == "KP") {
      Serial.print("Kp =   ");
      Kp = data * 1E-9;
      Serial.println(Kp,4);
    }
    if (command == "KD") {
      Serial.print("Kd =   ");
      Kd = data;
      Serial.println(Kd,4);
    }
    if (command == "KI") {
      Serial.print("Ki =   ");
      Ki = data * 1E-12;
      Serial.println(Ki,12);
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
      l_freq = ( 1/ elapsed_uS * 2.0) * 1E8; // centiHz
      left_last_uS = current_uS;
      left_timeout_uS = left_last_uS + SPEED_TIMEOUT;
      leftLastState = left_state;
    } else if (micros() > left_timeout_uS) {
        l_freq = l_min_freq;
        left_last_uS = micros();
    }
    // Check if the pin has changed state
    if (right_state != rightLastState) {
      // Calculate how long has passed since last transition
      unsigned long current_uS = micros();
      unsigned long elapsed_uS = current_uS - right_last_uS;

      // Calculate the frequency of the input signal
      r_freq = ( 1/ elapsed_uS * 2.0) * 1E8; // centiHz
      right_last_uS = current_uS;
      right_timeout_uS = right_last_uS + SPEED_TIMEOUT;
      rightLastState = right_state;
    } else if (micros() > right_timeout_uS) {
        r_freq = r_min_freq;
        right_last_uS = micros();
    }
      l_total = l_total - l[l_readIndex];
      l[l_readIndex] = l_freq;
      l_total = l_total + l[l_readIndex];
      l_readIndex ++;
      if(l_readIndex >= l_numReadings) l_readIndex = 0;
      l_freq_mean = l_total / l_numReadings;

      r_total = r_total - r[r_readIndex];
      r[r_readIndex] = r_freq;
      r_total = r_total + r[r_readIndex];
      r_readIndex ++;
      if(r_readIndex >= r_numReadings) r_readIndex = 0;
      r_freq_mean = r_total / r_numReadings;
}

// Writes the RPM and MPH to the serial port at a set interval
void WriteToSerial() {
    static unsigned long updateTime;
    
//    Serial.println("l_freq - l_freq_mean - l_sp - l_error - l_error_sum - l_pwm");
    if (millis() > updateTime) {
        Serial.print((String)millis() + ", ");
        Serial.print((String)l_freq + ", ");
        Serial.print((String)l_freq_mean + ", ");
        Serial.print((String)l_sp + ", ");
        Serial.print((String)l_err + ", ");
        Serial.print((String)l_err_sum + ", ");
        Serial.print((String)l_pwm + ",");

        Serial.print((String)r_freq + ", ");
        Serial.print((String)r_freq_mean + ", ");
        Serial.print((String)r_sp + ", ");
        Serial.print((String)r_err + ", ");
        Serial.print((String)r_err_sum + ", ");
        Serial.println((String)r_pwm);

        updateTime = millis() + UPDATE_TIME;
    }
}
