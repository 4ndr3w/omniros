#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "SerialPort.h"
#include "OmniBotComm.h"
#include "fletcher.h"

SerialPort<RobotCommand, RobotStatus> serial("/dev/ttyACM0", B9600);

// Radius in meters from the center of the robot to the wheel
double robotRadius = 0.1;

// Radius of the wheel
double wheelRadius = 0.0508;

void commandVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
  RobotCommand cmd;

  cmd.frontVelocity = robotRadius * -msg->angular.z;
  cmd.backVelocity = -robotRadius * msg->angular.z;
  cmd.leftVelocity = msg->linear.y + (robotRadius * -msg->angular.z);
  cmd.rightVelocity = -msg->linear.y - (robotRadius * msg->angular.z);

  cmd.checksum = 0;

  uint32_t checksum = fletcher16(&cmd, sizeof(RobotCommand));

  cmd.checksum = checksum;

  serial.sendMessage(cmd);
}

/*
 * robot_sender node
 * Forwards ROS velocity commands to the robot controller
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_sender");

  ros::NodeHandle n;

  ros::Subscriber velSub = n.subscribe("cmd_velocity", 100, commandVelocity);
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ROS_INFO("omnibot_differential_base node started");

  ros::Rate rate(120);
  while ( n.ok() ) {
    ros::Time poseTime = ros::Time::now();

    if ( serial.hasMessage() ) {
      RobotStatus robotPose = serial.getMessage();

      uint32_t checksum = robotPose.checksum;
      robotPose.checksum = 0;
      if ( fletcher16(&robotPose, sizeof(RobotStatus)) == checksum ) {
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPose.heading);
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = poseTime;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = robotPose.x;
        odom_trans.transform.translation.y = robotPose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = poseTime;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = robotPose.x;
        odom.pose.pose.position.y = robotPose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = robotPose.vx;
        odom.twist.twist.linear.y = robotPose.vy;
        odom.twist.twist.angular.z = robotPose.vth;

        //publish the message
        odom_pub.publish(odom);
      }
      else {
        ROS_ERROR("Checksum failed");
        serial.flush();
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
}
