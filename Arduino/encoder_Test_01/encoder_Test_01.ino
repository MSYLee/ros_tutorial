// motor control pin
const int motorDirPin = 4; // L298 Input 1
const int motorPWMPin = 5; // L298 Input 2

// encoder pin
const byte encoderPinA = 2;
const byte encoderPinB = 3;
//const boolean encoderStateA = False;

float motorDeg = 0;

int encoderPos = 0;
const float ratio = 360. / 34. / 52.;

// P control
float Kp = 30;
float targetDeg = 360;
int tmp = 0;



void doMotor(bool dir, int vel) {
  digitalWrite(motorDirPin, dir);
  analogWrite(motorPWMPin, dir ? (255 - vel) : vel);


}

void setup() {
  //  pinMode(encoderPinA, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  //
  //  pinMode(encoderPinB, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);

  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

  pinMode(motorDirPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {


//  if(encoderPos>1800){
//  //digitalWrite(motorDirPin, LOW);
//  analogWrite(motorPWMPin, 0);
//  Serial.print("encoderPos : ");
//  Serial.println(encoderPos);
//  delay(5000);
//  encoderPos = 0;
//    }
//    if(encoderPos<1800){
//  digitalWrite(motorDirPin, LOW);
//  analogWrite(motorPWMPin, 255);}
//    
//  


  /*
    Serial.print("     MotorDeg1 :");
    Serial.println(motorDeg);

    Serial.print("encoderPos1 :");
    Serial.println(encoderPos);
//  */
  float motorDeg = float(encoderPos) * ratio;
//
//  Serial.print("ratio : ");
//  Serial.print(ratio);
  float error = targetDeg - motorDeg;
  float control = Kp * error;
  /*
    Serial.print("     MotorDeg2 :");
    Serial.println(motorDeg);
    Serial.print("encoderPos2 :");
    Serial.println(encoderPos);
  */
  doMotor( (control >= 0) ? HIGH : LOW, min(abs(control), 255));
  /*
    Serial.print("     MotorDeg3 :");
    Serial.println(motorDeg);
    Serial.print("encoderPos3 :");
    Serial.println(encoderPos);
  */

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
  Serial.print("encoderPinA : ");
  Serial.println(digitalRead(encoderPinA));


}

void doEncoderA() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
  tmp += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;

}
void doEncoderB() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}
