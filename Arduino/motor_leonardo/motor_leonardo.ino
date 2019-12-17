#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 0.0;
double y = 0.0;
double theta = 1.57;
char base_link[] = "/base_link";
char odom[] = "/odom";
ros::NodeHandle  nh;
int RightMotorP = 5;
int RightMotorFNR = 4;
int LeftMotorP = 6;
int LeftMotorFNR = 7;
int RightsignalA = 2;
int RightsignalB = 3;
int LeftsignalA = 1;
int LeftsignalB = 0;
int way = 0;
double w_r = 0, w_l = 0;
double wheel_rad = 0.0275, wheel_sep = 0.15;
int highSpeed = 255;
int lowSpeed = 170;
double speed_ang = 0, speed_lin = 0;


int R_encoderPos = 0, L_encoderPos = 0;


//http://wiki.ros.org/rosserial_arduino/Tutorials
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); // 1.


void TwistCallback(const geometry_msgs::Twist& msg) {
  if (msg.angular.z > 0) {
    digitalWrite(LeftMotorFNR, HIGH);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, HIGH);
    analogWrite(RightMotorP, 255);
    way = 3;
  }
  if (msg.angular.z < 0) {
    digitalWrite(LeftMotorFNR, LOW);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, LOW);
    analogWrite(RightMotorP, 255);
    way = 4;
  }
  if (msg.linear.x > 0) {
    digitalWrite(LeftMotorFNR, HIGH);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, LOW);
    analogWrite(RightMotorP, 255);
    way = 1;
  }
  if (msg.linear.x < 0) {
    digitalWrite(LeftMotorFNR, LOW);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, HIGH);
    analogWrite(RightMotorP, 255);
    way = 2;
  }
  if (msg.linear.x == 0) {
    analogWrite(RightMotorP, 0);
    analogWrite(LeftMotorP, 0);
    way = 0;
  }
  /*  char Pub[32] = {0};
    PubWidth.toCharArray(Pub, PubWidth.length());
    str_msg.data = Pub;
    chatter.publish(&str_msg);
    PubWidth = String(w_l);
    PubWidth.toCharArray(Pub, PubWidth.length());
    str_msg.data = Pub;
    chatter.publish(&str_msg);*/
}
ros::Subscriber<geometry_msgs::Twist> sub("MyRobot/cmd_vel", TwistCallback ); //2.

void setup() {

  broadcaster.init(nh);
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
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

  Serial.begin(115200);
}
unsigned long current_time, prev_time;

bool stop_r = false, stop_l = false;
void loop() {
  // drive in a circle



    nh.spinOnce();


  // put your main code here, to run repeatedly:


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
