
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define USE_USBCON
#define LEFT                             0
#define RIGHT                            1
#define WHEEL_NUM                        2
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";

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
char log_msg;
double wheel_l, wheel_r;
double delta_s, delta_theta, theta, last_theta;


double R_encoderPos = 0, L_encoderPos = 0;
float odom_pose[3];


//http://wiki.ros.org/rosserial_arduino/Tutorials
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); // 1.

ros::Time current_time;
uint32_t current_offset;


nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);


void TwistCallback(const geometry_msgs::Twist& msg) {
  if (msg.linear.x > 0) {
    digitalWrite(LeftMotorFNR, HIGH);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, HIGH);
    analogWrite(RightMotorP, 255);
    way = 3;
  }
  if (msg.linear.x < 0) {
    digitalWrite(LeftMotorFNR, LOW);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, LOW);
    analogWrite(RightMotorP, 255);
    way = 4;
  }
  if (msg.angular.z > 0) {
    digitalWrite(LeftMotorFNR, HIGH);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, LOW);
    analogWrite(RightMotorP, 255);
    way = 1;

  }
  if (msg.angular.z < 0) {
    digitalWrite(LeftMotorFNR, LOW);
    analogWrite(LeftMotorP, 255);
    digitalWrite(RightMotorFNR, HIGH);
    analogWrite(RightMotorP, 255);
    way = 2;

  }
  if (msg.linear.x == 0 && msg.angular.z == 0) {
    analogWrite(RightMotorP, 0);
    analogWrite(LeftMotorP, 0);


  }

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

  nh.getHardware()->setBaud(115200);
 
}


bool stop_r = false, stop_l = false;





void loop() {


   nh.spinOnce();

  
  



}



void updateOdometry(void)
{
  

  wheel_l = 0.09594 * (double)L_encoderPos;
  wheel_r = 0.09594 * (double)R_encoderPos;

  delta_s     = 0.055 * (wheel_r + wheel_l) / 2.0;
  theta = 0.055 * (wheel_r - wheel_l);

  delta_theta = theta - last_theta;

  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;


  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = 0;
  odom.twist.twist.angular.z = 0;

  last_theta = theta;
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
