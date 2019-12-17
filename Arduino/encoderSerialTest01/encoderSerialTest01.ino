int encoderPosRight = 0;
int RightMotorP = 5;
int RightMotorFNR = 4;
int RightsignalA = 8;
int RightsignalB = 9;

void setup() {
  
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(RightsignalA, INPUT_PULLUP);
  pinMode(RightsignalB, INPUT_PULLUP);
  
  pinMode(RightMotorFNR, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
 //digitalWrite(LeftMotorFNR,HIGH);
 //analogWrite(LeftMotorP,120);
 digitalWrite(RightMotorFNR, HIGH);
 analogWrite(RightMotorP,0);
 Serial.print("  A : ");
 Serial.print(digitalRead(RightsignalA));
 Serial.print("  B : ");
 Serial.println(digitalRead(RightsignalB));
 delay(10);
}
