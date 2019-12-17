/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
std_msgs::String robot_state;
ros::Publisher ROBOT_status("ROBOT_status",&robot_state);
int RightMotorP = 6;
int RightMotorFNR = 7;
int LeftMotorP = 5;
int LeftMotorFNR = 4;
int RightsignalA = 8;
int RightsignalB = 9;
int LeftsignalA = 10;
int LeftsignalB = 11;

String state;

void robot_FWD(){
    digitalWrite(RightMotorFNR,HIGH);
    analogWrite(RightMotorP, 500);
    digitalWrite(LeftMotorFNR,LOW);
    analogWrite(LeftMotorP, 500);
    delay(1500);
    analogWrite(RightMotorP, 0);
    analogWrite(LeftMotorP, 0);
  }
void robot_RWD() {
  
    digitalWrite(RightMotorFNR,LOW);
    analogWrite(RightMotorP, 100);
    digitalWrite(LeftMotorFNR,HIGH);
    analogWrite(LeftMotorP, 100);
    delay(1500);
    analogWrite(RightMotorP, 0);
    analogWrite(LeftMotorP, 0);
  }

void robot_STP(){
  
    analogWrite(RightMotorP, 0);
    analogWrite(LeftMotorP, 0);
  }
void action_ROBOT( const std_msgs::String& msg){
  state = msg.data;
  ROBOT_status.publish(&msg);
  if (state=="go") {
    robot_FWD();
  }

  if (state=="back") {
    robot_RWD();
    }
  if (state=="stop"){
    robot_STP();
    }
}
ros::Subscriber<std_msgs::String> sub("robot_control", &action_ROBOT );

void setup()
{
  pinMode(RightMotorFNR, OUTPUT);
  pinMode(LeftMotorFNR, OUTPUT);
  
  nh.initNode();
  nh.advertise(ROBOT_status);
  nh.subscribe(sub);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop()
{
  nh.spinOnce();
  delay(1000);
}
