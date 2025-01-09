#define V_PIN 3
#define DIR_PIN 2

void setup() {
  Serial.begin(9600);
  pinMode(V_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void loop() {
  testMotor();
  delay(1000);
  
}

void testMotor(){
  Serial.println("Moving Right");
  digitalWrite(DIR_PIN, LOW);
  for(int v = 0; v < 255; v++){
    analogWrite(V_PIN, v);
    delay(10);
  }
  for(int v = 255; v >=0; v--){
    analogWrite(V_PIN, v);
    delay(10);
  }
  Serial.println("Moving Left");
  digitalWrite(DIR_PIN, HIGH);
  for(int v = 0; v < 255; v++){
    analogWrite(V_PIN, v);
    delay(10);
  }
  for(int v = 255; v >=0; v--){
    analogWrite(V_PIN, v);
    delay(10);
  }
}
