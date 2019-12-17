int encoderPosRight = 0;
int encoderPosLeft = 0;
int RightMotorP = 6;
int RightMotorFNR = 7;
int LeftMotorP = 5;
int LeftMotorFNR = 4;
int RightsignalA = 8;
int RightsignalB = 9;
int LeftsignalA = 10;
int LeftsignalB = 11;
void doEncoderRightA(){ // 빨녹일 때
  if(digitalRead(RightsignalA)==digitalRead(RightsignalB)) // 같으면
    encoderPosRight++; // 정회전
  else // 다르면
    encoderPosRight--; // 역회전
  //encoderPosRight += (digitalRead(RightsignalA)==digitalRead(RightsignalB))?1:-1;
  Serial.print("A Right :  ");
  Serial.println(encoderPosRight);
}
//void doEncoderLeftA(){ // 빨녹일 때
////  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
////    encoderPos++; // 정회전
////  else // 다르면
////    encoderPos--; // 역회전
//  encoderPosLeft += (digitalRead(LeftsignalA)==digitalRead(LeftsignalB))?1:-1;
//  Serial.print("A Left :  ");
//  Serial.println(encoderPosLeft);
//}

void doEncoderRightB(){ // 보파일 때
  if(digitalRead(RightsignalA)==digitalRead(RightsignalB)) // 같으면
    encoderPosRight--; // 역회전
  else // 다르면
    encoderPosRight++; // 정회전
//  encoderPosRight += (digitalRead(RightsignalA)==digitalRead(RightsignalB))?-1:1;
  Serial.print("B Right :  ");
  Serial.println(encoderPosRight);
}
//void doEncoderLeftB(){ // 보파일 때
////  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
////    encoderPos--; // 역회전
////  else // 다르면
////    encoderPos++; // 정회전
//  encoderPosLeft += (digitalRead(LeftsignalA)==digitalRead(LeftsignalB))?-1:1;
//  Serial.print("B Left : ");
//  Serial.println(encoderPosLeft);
//}
//

void setup() {
  
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(RightsignalA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderRightA, CHANGE);
//  pinMode(LeftsignalA, INPUT_PULLUP);
//  attachInterrupt(0, doEncoderLeftA, CHANGE);
  pinMode(RightsignalB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderRightB, CHANGE);
//  pinMode(LeftsignalB, INPUT_PULLUP);
//  attachInterrupt(1, doEncoderLeftB, CHANGE);
// 
  //EncoderInit();
  pinMode(LeftMotorFNR, OUTPUT);
  pinMode(RightMotorFNR, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(LeftMotorFNR,LOW);
 analogWrite(LeftMotorP,0);
 digitalWrite(RightMotorFNR,HIGH);
 analogWrite(RightMotorP,120);
 Serial.print("A : ");
 Serial.println(digitalRead(RightsignalA));
 //Serial.print("B : ");
 //Serial.println(digitalRead(RightsignalB));
 //Serial.println(LeftsignalA);
 //Serial.println(LeftsignalB);
}
