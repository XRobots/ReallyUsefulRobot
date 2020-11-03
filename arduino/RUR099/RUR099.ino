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

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;

char base_link[] = "/base_link";
char odom[] = "/odom";

// cmd_vel variables to be received to drive with
float demandx;
float demandz;

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float loopTime = 10;

// ODrive init stuff
int button;
int requested_state;

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

// ** ROS callback & subscriber **

void velCallback(  const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);                    

// ** Setup **

void setup() {

  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  nh.advertise(odom_pub);
  broadcaster.init(nh);       // set up broadcaster

  pinMode(2, INPUT_PULLUP);   // ODrive init switch

  Serial1.begin(115200);    // ODrive
  Serial6.begin(115200);    // debug port using a USB-serial adapter (Serial-zero is in use by ros_serial

}

// ** Main loop **

void loop() {

  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  currentMillis = millis();
        if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms          
            previousMillis = currentMillis;          // reset the clock to time it

            button = digitalRead(2);                 // init ODrive
            if (button == 0) {
              OdriveInit1();
            } 

            float modifier_lin = 1.03;        // scaling factor because the wheels are squashy / there is wheel slip etc.
            float modifier_ang = 0.92;        // scaling factor because the wheels are squashy / there is wheel slip etc.


            forward0 = demandx * (83466 * modifier_lin) ; // convert m/s into counts/s
            forward1 = demandx * (83466 * modifier_lin); // convert m/s into counts/s

            turn0 = demandz * (15091 * modifier_ang);    // convert rads/s into counts/s
            turn1 = demandz * (15091 * modifier_ang);    // convert rads/s into counts/s

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
            float phi = ((pos1_mm_diff - pos0_mm_diff) / 360);

            theta += phi;
          
            if (theta >= TWO_PI) {
                theta -= TWO_PI;
            }
            if (theta <= (-TWO_PI)) {
                theta += TWO_PI;
            }

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
            odom_msg.twist.twist.linear.y = 0.0;                                        // robot does not move sideways
            odom_msg.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) / 360)*100;      // anglular velocity

            odom_pub.publish(&odom_msg);

        } // end of 10ms loop

} // end of main loop
