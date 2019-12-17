#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
int RightMotorP = 5;
int RightMotorFNR = 4;
int LeftMotorP = 6;
int LeftMotorFNR = 7;
int RightsignalA = 2;
int RightsignalB = 3;
int LeftsignalA = 1;
int LeftsignalB = 0;

double w_r = 0, w_l = 0;
double wheel_rad = 0.0275, wheel_sep = 0.15;
int highSpeed = 255;
int lowSpeed = 170;
double speed_ang = 0, speed_lin = 0;


int R_encoderPos = 0, L_encoderPos = 0;

/*
  //http://wiki.ros.org/rosserial_arduino/Tutorials
  std_msgs::String str_msg;
  ros::Publisher chatter("chatter", &str_msg); // 1.
  void TwistCallback(const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  //w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  //w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));

  w_r = speed_lin + speed_ang * (wheel_sep) / 2;
  w_l = speed_lin - speed_ang * (wheel_sep) / 2;
  String PubWidth = String(w_r);
  char Pub[32] = {0};
  PubWidth.toCharArray(Pub, PubWidth.length());
  str_msg.data = Pub;
  chatter.publish(&str_msg);
  PubWidth = String(w_l);
  PubWidth.toCharArray(Pub, PubWidth.length());
  str_msg.data = Pub;
  chatter.publish(&str_msg);
  }


  void MotorL(int Pulse_Width1) {
  String PubWidth = String(Pulse_Width1);
  char Pub[32] = {0};
  PubWidth.toCharArray(Pub, PubWidth.length());
  str_msg.data = Pub;
  chatter.publish(&str_msg);
  if (Pulse_Width1 > 0) {

    analogWrite(LeftMotorP, Pulse_Width1);
    digitalWrite(LeftMotorFNR, LOW);

  }
  if (Pulse_Width1 < 0) {
    Pulse_Width1 = abs(Pulse_Width1);
    analogWrite(LeftMotorP, Pulse_Width1);
    digitalWrite(LeftMotorFNR, HIGH);
  }
  if (Pulse_Width1 == 0) {

    analogWrite(LeftMotorP, Pulse_Width1);
    digitalWrite(LeftMotorFNR, LOW);
  }
  }

  void MotorR(int Pulse_Width2) {
  String PubWidth = String(Pulse_Width2);
  char Pub[32] = {0};
  PubWidth.toCharArray(Pub, PubWidth.length());
  str_msg.data = Pub;
  chatter.publish(&str_msg);
  if (Pulse_Width2 > 0) {

    analogWrite(RightMotorP, Pulse_Width2);
    digitalWrite(RightMotorFNR, HIGH);
  }
  if (Pulse_Width2 < 0) {


    Pulse_Width2 = abs(Pulse_Width2);
    analogWrite(RightMotorP, Pulse_Width2);
    digitalWrite(RightMotorFNR, LOW);
  }
  if (Pulse_Width2 == 0) {
    analogWrite(RightMotorP, Pulse_Width2);
    digitalWrite(RightMotorFNR, HIGH);
  }

  }
  ros::Subscriber<geometry_msgs::Twist> sub("MyRobot/cmd_vel", TwistCallback ); //2.
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(RightsignalA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RightsignalB) , RightEncoderA, CHANGE);
  pinMode(RightsignalB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RightsignalA), RightEncoderB, CHANGE);



  pinMode(LeftsignalA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LeftsignalB), LeftEncoderA, CHANGE);
  pinMode(LeftsignalB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LeftsignalA), LeftEncoderB, CHANGE);
  pinMode(RightMotorFNR, OUTPUT);
  pinMode(LeftMotorFNR, OUTPUT);
  /*nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);*/

  Serial.begin(115200);
}
unsigned long current_time, prev_time;

bool stop_r = false, stop_l = false;
void loop() {
  // put your main code here, to run repeatedly:
  
    if (L_encoderPos <= 1800) {
      if (!stop_l) {
        digitalWrite(LeftMotorFNR, HIGH);
        analogWrite(LeftMotorP, 255);
        

      }
    }
    if (L_encoderPos >= 1800) {

      analogWrite(LeftMotorP, 0);
      L_encoderPos = 0;
      current_time = millis();
      Serial.println(current_time);
      stop_l = true;
    }

    if (R_encoderPos <= 1800) {
      if (!stop_r) {
        digitalWrite(RightMotorFNR, LOW);
        analogWrite(RightMotorP, 255);
      }

    }
    if (R_encoderPos >= 1800) {

      analogWrite(RightMotorP, 0);
      R_encoderPos = 0;
      stop_r = true;
    }
    if (stop_l && stop_r)
    {

      Serial.print("R_encoderPos :");
      Serial.println(R_encoderPos);

      Serial.print("L_encoderPos :");
      Serial.println(L_encoderPos);
      delay(5000);

      stop_l = false;
      stop_r = false;
    }

  /*
    nh.spinOnce();
    String PubWidth = String(w_r*10);
    char Pub[32] = {0};
    PubWidth.toCharArray(Pub, PubWidth.length());
    str_msg.data = Pub;
    chatter.publish(&str_msg);
    PubWidth = String(w_l*10);
    PubWidth.toCharArray(Pub, PubWidth.length());
    str_msg.data = Pub;
    chatter.publish(&str_msg);
    MotorL(w_l*10);
    MotorR(w_r*10);
    delay(1000);*/
}

void RightEncoderA() {
  R_encoderPos += (digitalRead(RightsignalA) == digitalRead(RightsignalB)) ? 1 : -1;

}
void RightEncoderB() {
  R_encoderPos += (digitalRead(RightsignalA) == digitalRead(RightsignalB)) ? -1 : 1;
}


void LeftEncoderA() {
  L_encoderPos += (digitalRead(LeftsignalA) == digitalRead(LeftsignalB)) ? 1 : -1;

}
void LeftEncoderB() {
  L_encoderPos += (digitalRead(LeftsignalA) == digitalRead(LeftsignalB)) ? -1 : 1;
}
