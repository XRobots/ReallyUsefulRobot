#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial2

//ODrive
#include <ODriveArduino.h>

//ODrive Objects
ODriveArduino odrive1(Serial1);

//ROS
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

std_msgs::UInt16 eye_msg;
ros::Publisher pub_eye("eye", &eye_msg);

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;

float xArm;
float yArm;
float zArm;

char arm[] = "/arm";
char base_link[] = "/base_link";
char odom[] = "/odom";

// cmd_vel variables to be received to drive with
float demandx;
float demandz;

float demandArmX;           // demand values from sticks
float demandArmY;
float demandArmZ;  

float demandEye;        

float demandArmZAccum;      // accumulators for manual mode
float demandArmXAccum;
float demandArmYAccum;
float demandArmXAccum2;
float demandArmYAccum2;

float demandEyeAccum;
float demandEyeAccum2;

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float loopTime = 10;

// ODrive init stuff
int button1;
int button2;
int prox;
int requested_state;

int mode;
int modeOld;
int homeFlag = 0;

// output variables to drive the ODrive
int forward0;
int forward1;
int turn0;
int turn1;

// position and velocity variables read from the ODrive
int vel0;
int vel1;
long pos0;
long pos1;

// variables to work out the different on each cycle

long pos0_old;
long pos1_old;
long pos0_diff;
long pos1_diff;
float pos0_mm_diff;
float pos1_mm_diff;
float pos_average_mm_diff;
float pos_total_mm;

// encoder stuff

#define encoder0PinA 18      // encoder 1
#define encoder0PinB 19

volatile long encoder0Pos = 0;    // encoder 1

// arm variables

const uint8_t DXL_DIR_PIN = 21; // DYNAMIXEL Shield DIR PIN 
const float DXL_PROTOCOL_VERSION = 2.0;

int offset10 = 90;   // 90 folded
int offset20 = 180;  // 270 folded
int offset30 = 180;  // 90 folded
int offset40 = 180;  // 180 mid
int offset50 = 180;  // 180 mid

float pos10;
float pos20;
float pos30;
float pos40;
float pos50;
float pos50Pub;

float finPos10;
float finPos20;
float finPos30;
float finPos40;
float finPos50;

float finPos10Prev;
float finPos20Prev;
float finPos30Prev;
float finPos40Prev;
float finPos50Prev;

float armX = 350;         // variables received from Vector3 message to drive arm
float armY = 145;
float armZ = 0;

int pot1;   // test pots from video part 4
int pot2;

long previousFoldMillis = 0;
int fold = 0;
int foldFlag = 0;

// moving the foot forwards or backwards in the side plane
float shoulderAngle2;
float shoulderAngle2a;
float shoulderAngle2Degrees;
float shoulderAngle2aDegrees;
float y3;
float y2;
float x3;
float x2;

// side plane of individual leg only
#define lowerLength 370     
#define upperLength 238
float armLength;
float shoulderAngle1;
float shoulderAngle1Degrees;
float shoulderAngle1a;   
float elbowAngle1;
float elbowAngle1a;
float elbowAngle1Degrees;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// ** ROS callback & subscriber **

void velCallback(  const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;
          
     demandArmX = vel.angular.x;
     demandArmY = vel.angular.y;
     demandArmZ = vel.linear.z;

     demandEye = vel.linear.y;
}

void foldArm( const std_msgs::UInt16& cmd_msg){
  mode = (cmd_msg.data);
  if (mode != modeOld) {
    if (mode == 1) {
      fold = 1;
    }
    else if (mode == 2) {
      fold = 0;
    }
  }
  modeOld = mode;
}

void armCoords( const geometry_msgs::Vector3 cmd_msg){
  armX = (cmd_msg.x);
  armY = (cmd_msg.y);
  armZ = (cmd_msg.z);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);  
ros::Subscriber<std_msgs::UInt16> sub2("buttons", foldArm);
ros::Subscriber<geometry_msgs::Vector3> sub3("armCoords", armCoords);

// ** Setup **

void setup() {

  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  nh.subscribe(sub2);          // subscribe to arm fold
  nh.subscribe(sub3);          // subscribe to Vector3 arm messages
  nh.advertise(odom_pub);
  broadcaster.init(nh);       // set up broadcaster
  nh.advertise(pub_eye);   // advertise eye topic

  pinMode(2, INPUT_PULLUP);   // ODrive init switch
  pinMode(3, INPUT_PULLUP);   // linear axis home switch

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(14, OUTPUT);    // linear motor drive
  pinMode(15, OUTPUT);

  pinMode(41, INPUT_PULLUP);  // proximity switch

  attachInterrupt(18, doEncoderA, CHANGE);
  attachInterrupt(19, doEncoderB, CHANGE);

  Serial1.begin(115200);    // ODrive
  Serial6.begin(115200);    // debug port using a USB-serial adapter (Serial-zero is in use by ros_serial

  // Dynamixel setup

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(10);
  dxl.ping(20);
  dxl.ping(30);
  dxl.ping(40);
  dxl.ping(50);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(10);
  dxl.torqueOff(20);
  dxl.torqueOff(30);
  dxl.torqueOff(40);
  dxl.torqueOff(50);
  dxl.setOperatingMode(10, OP_POSITION);
  dxl.setOperatingMode(20, OP_POSITION);
  dxl.setOperatingMode(30, OP_POSITION);
  dxl.setOperatingMode(40, OP_POSITION);
  dxl.torqueOn(10);
  dxl.torqueOn(20);
  dxl.torqueOn(30);
  dxl.torqueOn(40);
  dxl.torqueOn(50);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, 10, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 20, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 30, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 40, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, 50, 30);

  fold = 1;   // start with arm folded out for testing

}

// ** Main loop **

void loop() {

  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  currentMillis = millis();
        if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms          
            previousMillis = currentMillis;          // reset the clock to time it

            button1 = digitalRead(2);                 // init ODrive
            if (button1 == 0) {
              OdriveInit1();
            }  

            button2 = digitalRead(3);                 // home linear axis
            prox = digitalRead(41);                   // proximity switch

            //demandArmZAccum = demandArmZAccum + (demandArmZ * 160);   /// add or subtract to the value on each cycle.
            //demandArmZAccum = constrain (demandArmZAccum, -50000,0);
            demandArmZAccum = armZ;   // use data from Vector3 message instead
            demandArmZAccum = constrain (demandArmZAccum, -50000,0);

            if  (homeFlag == 0 && button2 == 0) {          
              analogWrite(14, 100);       
              analogWrite(15, 0);         // drive axis up toward proximity sensor
              homeFlag = 1;
            }

            else if (homeFlag == 1 && prox == 0) {
              analogWrite(14, 0);           // stop motor
              analogWrite(15, 0);
              encoder0Pos = 0;              // zero encoder
              homeFlag = 2;
            }  

            if (prox == 0) {
              encoder0Pos = 0;              // always zero encoder when it hits to the top
            }

            // drive motor

            if (homeFlag == 2) {

                if (demandArmZAccum < encoder0Pos - 50) {
                  analogWrite(14, 0);       
                  analogWrite(15, 100);         // drive axis down
                }
                else if (demandArmZAccum > encoder0Pos + 50) {
                  analogWrite(14, 100);       
                  analogWrite(15, 0);         // drive axis down
                }
    
                else {
                  analogWrite(14, 0);       
                  analogWrite(15, 0);         // stop                             
                }

            }

            // deal with driving wheels

            forward0 = demandx * 83466; // convert m/s into counts/s
            forward1 = demandx * 83466; // convert m/s into counts/s

            turn0 = demandz * 15091;    // convert rads/s into counts/s
            turn1 = demandz * 15091;    // convert rads/s into counts/s

            forward1 = forward1*-1;      // one motor and encoder is mounted facing the other way

            odrive1.SetVelocity(0, forward0 + turn0); 
            odrive1.SetVelocity(1, forward1 + turn1);

            // get positions and velocities from ODrive

            pos0 = (odrive1.GetPosition(1)) *-1;                   
            pos1 = odrive1.GetPosition(0);   

            // work out the difference on each loop, and bookmark the old value
            pos0_diff = pos0 - pos0_old;
            pos1_diff = pos1 - pos1_old;            
            pos0_old = pos0;
            pos1_old = pos1;
    
            // calc mm from encoder counts
            pos0_mm_diff = pos0_diff / 83.44;
            pos1_mm_diff = pos1_diff / 83.44;

            // calc distance travelled based on average of both wheels
            pos_average_mm_diff = (pos0_mm_diff + pos1_mm_diff) / 2;   // difference in each cycle
            pos_total_mm += pos_average_mm_diff;                       // calc total running total distance

            // calc angle or rotation to broadcast with tf
            theta += (pos1_mm_diff - pos0_mm_diff) / 360;
            
            if (theta > PI)
            theta -= TWO_PI;
            if (theta < (-PI))
            theta += TWO_PI;

            // calc x and y to broadcast with tf

            y += pos_average_mm_diff * sin(theta);
            x += pos_average_mm_diff * cos(theta);

            // *** broadcast odom->base_link transform with tf ***

            geometry_msgs::TransformStamped t;           
                      
            t.header.frame_id = odom;
            t.child_frame_id = base_link;
            
            t.transform.translation.x = x/1000;   // convert to metres
            t.transform.translation.y = y/1000;
            t.transform.translation.z = 0;
            
            t.transform.rotation = tf::createQuaternionFromYaw(theta);
            t.header.stamp = nh.now();
                     
            broadcaster.sendTransform(t);

            // *** broadcast base_link->arm transform with tf ***

            geometry_msgs::TransformStamped k;

            k.header.frame_id = base_link;
            k.child_frame_id = arm;

            zArm = 1.0 + ((0.5/50000) * encoder0Pos);            
           
            k.transform.translation.x = (x3/1000)-0.1;  
            k.transform.translation.y = (y3/1000)*-1;
            k.transform.translation.z = zArm; 

            k.transform.rotation = tf::createQuaternionFromYaw(0);

            broadcaster.sendTransform(k);
            k.header.stamp = nh.now();

            // *** broadcast odom message ***

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = nh.now();
            odom_msg.header.frame_id = odom;
            odom_msg.pose.pose.position.x = x/1000;
            odom_msg.pose.pose.position.y = y/1000;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

            odom_msg.child_frame_id = base_link;
            odom_msg.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) / 2)/10;          // forward linear velovity
            odom_msg.twist.twist.linear.y = 0.0;                                            // robot does not move sideways
            odom_msg.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) / 360)*100;      // anglular velocity

            odom_pub.publish(&odom_msg);

            // Arm Control

            // unfold        

            if (fold == 1 && foldFlag == 0) {
                pos30 = 126;
                foldFlag = 1;
                previousFoldMillis = currentMillis;          
            }
            else if (fold == 1 && foldFlag == 1 && currentMillis - previousFoldMillis >= 1000) {
                pos10 = 45;
                pos20 = 135;
                foldFlag = 2;
                previousFoldMillis = currentMillis;
            }
            else if (fold == 1 && foldFlag == 2 && currentMillis - previousFoldMillis >= 2000) {
                // wait for unfold
                foldFlag = 0;
                fold = 5;
            }
    
            // fold
    
            if (fold == 0 && foldFlag == 0) {
                pos10 = 0;
                pos30 = 126;
                foldFlag = 1;
                previousFoldMillis = currentMillis; 
            }
    
            else if (fold == 0 && foldFlag == 1 && currentMillis - previousFoldMillis >= 1000) {
                pos10 = 0;
                pos20 = 90;
                foldFlag = 2;
                previousFoldMillis = currentMillis;
            }
            
            else if (fold == 0 && foldFlag == 2 && currentMillis - previousFoldMillis >= 1000) {
                pos30 = 90;
                foldFlag = 0;
                fold = 2;
            }

            // arm data hendling and kinematics

            Serial6.print(armX);      // Vector3 messages received
            Serial6.print(" , ");
            Serial6.print(armY);
            Serial6.print(" , ");
            Serial6.print(armZ);  
            Serial6.print(" *** ");     
            Serial6.print(x3);
            Serial6.print(" , ");
            Serial6.print(y3);
            Serial6.print(" , ");
            Serial6.print(demandArmZAccum);

            Serial6.println();
                 

            //demandArmXAccum = demandArmXAccum + (demandArmX * 10);   /// add or subtract to the value on each cycle.
            //demandArmXAccum = constrain(demandArmXAccum,-300,300);
            //x3 = map(demandArmXAccum, -300,300,580,350);
            x3 = armX;    // use Vector3 message from vision/depth instead
            x3 = constrain(x3,350,580);

            //demandArmYAccum = demandArmYAccum + (demandArmY * 10);   /// add or subtract to the value on each cycle.
            //y3 = constrain(demandArmYAccum,-300,300);
            y3 = armY;    // use Vector3 message from vision/depth instead
            y3 = constrain(y3,-300,300);

            if (fold == 5) {        // only run kinematics if arm is folded out
    
                  //dxl.writeControlTableItem(PROFILE_VELOCITY, 10, 60);
                  //dxl.writeControlTableItem(PROFILE_VELOCITY, 20, 60);
                  //dxl.writeControlTableItem(PROFILE_VELOCITY, 30, 60);
                  //dxl.writeControlTableItem(PROFILE_VELOCITY, 40, 60);
    
                  // calculate modification to shoulder angle and arm length
          
                  shoulderAngle2a = atan(y3/x3);
                  shoulderAngle2aDegrees = shoulderAngle2a * (180/PI);    // degrees
    
                  x2 = x3/cos(shoulderAngle2a);
    
                  // calculate arm length based on lower arm / upper arm length and elbow and shoulder angle
                  shoulderAngle1a = (sq(upperLength) + sq(x2) - sq(lowerLength)) / (2 * upperLength * x2);
                  shoulderAngle1 = acos(shoulderAngle1a);     // radians
                  elbowAngle1a = (sq(upperLength) + sq(lowerLength) - sq(x2)) / (2 * upperLength * lowerLength);
                  elbowAngle1 = acos(elbowAngle1a);     // radians
              
                  // calc degrees from angles
                  shoulderAngle1Degrees = shoulderAngle1 * (180/PI);    // degrees
                  elbowAngle1Degrees = elbowAngle1 * (180/PI);              // degrees      
          
                  pos10 = shoulderAngle1Degrees + shoulderAngle2aDegrees;
                  pos20 = elbowAngle1Degrees + 27;  // add on the offset cuased by the lower two sections of the arm being at 45'
    
                  pos40 = (270 - ((shoulderAngle1Degrees + shoulderAngle2aDegrees) + (elbowAngle1Degrees+27) + 126))*-1;
    
            }   // end of kinematics 

            // control the Eye tilt

            demandEyeAccum = demandEyeAccum + (demandEye * 10);   // add or subtract to the value on each cycle.
            demandEyeAccum = constrain(demandEyeAccum,-300,300);

            pos50 = map(demandEyeAccum,-300,300,-90,90);                     // constrain to the limits we want the eye to turn in degrees
            
            // constrain joints so we can't break the robot
            
            pos10 = constrain(pos10,0,90);
            pos20 = constrain(pos20,90,180);
            pos30 = constrain(pos30,90,180);
            pos40 = constrain(pos40,-90,90);
            pos50 = constrain(pos50,-90,90);
       
            //turn all the joints around so we can give the joint the actual angle, taking into account the servo offsets for default positions.
          
            finPos10 = offset10 + pos10;
            finPos20 = offset20 + (180-pos20);
            finPos30 = offset30 - (180-pos30);
            finPos40 = offset40 + pos40;
            finPos50 = offset50 + pos50;

          
            // write the joint positions to the servos, but only if they change.
            // the ROSserial library needs lots of time or it times out, so we keep other things to a minimum.

            if (finPos10 != finPos10Prev) {          
              dxl.setGoalPosition(10, finPos10, UNIT_DEGREE);
            }

            if (finPos20 != finPos20Prev) {          
              dxl.setGoalPosition(20, finPos20, UNIT_DEGREE);
            }

            if (finPos30 != finPos30Prev) {          
              dxl.setGoalPosition(30, finPos30, UNIT_DEGREE);
            }

            if (finPos40 != finPos40Prev) {          
              dxl.setGoalPosition(40, finPos40, UNIT_DEGREE);
            }

            if (finPos50 != finPos50Prev) {          
              dxl.setGoalPosition(50, finPos50, UNIT_DEGREE);
            }

            // bookmark previous values so we know if they changed

            finPos10Prev = finPos10;
            finPos20Prev = finPos20;
            finPos30Prev = finPos30;
            finPos40Prev = finPos40;
            finPos50Prev = finPos50;

            // publish eye angle as a ROStopic/message

            pos50Pub = dxl.getPresentPosition(50, UNIT_DEGREE);   // read actual angle from servo

            pos50Pub = map(pos50Pub,90,270,180,0);
            eye_msg.data = pos50Pub;
            pub_eye.publish(&eye_msg);                            // publish as ROS message



        } // end of timed loop

} // end of main loop


void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}
