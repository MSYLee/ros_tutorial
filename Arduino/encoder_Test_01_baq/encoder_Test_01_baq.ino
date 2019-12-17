// motor control pin
const int motorDirPin = 5; // L298 Input 1
const int motorPWMPin = 4; // L298 Input 2

// encoder pin
const byte encoderPinA = 2;
const byte encoderPinB = 3;
//const boolean encoderStateA = False;

int encoderPos = 0;
const float ratio = 360./34./26.;

// P control
float Kp = 30;
float targetDeg = 0;




void doMotor(bool dir, int vel){
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir?(255 - vel):vel);


}

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
 
  pinMode(motorDirPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  float motorDeg = float(encoderPos)*ratio;
 
  float error = targetDeg - motorDeg;
  float control = Kp*error;

  //doMotor( (control>=0)?HIGH:LOW, min(abs(control), 255));

 /*
  Serial.print("encoderPos : ");
  Serial.print(encoderPos);
  Serial.print("   motorDeg : ");
  Serial.print(float(encoderPos)*ratio);
  Serial.print("   error : ");
  Serial.print(error);
  Serial.print("    control : ");
  Serial.print(control);
  Serial.print("    motorVel : ");
  Serial.println(min(abs(control), 255));
  */

  Serial.print("encoderPinA : ");
  Serial.println(digitalRead(encoderPinA));


}

void doEncoderA(){ // 빨녹일 때
  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
    encoderPos++; // 정회전
  else // 다르면
    encoderPos--; // 역회전
  
  Serial.println("-------------doEncoderA-------------");
}

void doEncoderB(){ // 보파일 때
  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
    encoderPos--; // 역회전
  else // 다르면
    encoderPos++; // 정회전
  Serial.println("-------------doEncoderB-------------");
}
