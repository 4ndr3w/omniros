#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <omnibot_sim/reset_pose.h> 


double x, y, heading;

double velocity, omega;

double hz = 100.0;
double period = 1.0/hz;

void commandVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
    velocity = msg->linear.y;
    omega = msg->angular.z;

    ROS_INFO("Commanded %2.2f %2.2f", velocity, omega);
}

bool resetPoseService(omnibot_sim::reset_pose::Request& req, omnibot_sim::reset_pose::Response& res) {
  x = y = heading = velocity = omega = 0;
  return true;
}

/*
 * robot_sender node
 * Forwards ROS velocity commands to the robot controller
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_sender");

  ros::NodeHandle n;

  ros::ServiceServer resetPose = n.advertiseService("reset_pose", resetPoseService);
  ros::Subscriber velSub = n.subscribe("cmd_velocity", 100, commandVelocity);
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ROS_INFO("omnibot_differential_base SIM node started");

  ros::Rate rate(hz);

  while ( n.ok() ) {
    ros::Time poseTime = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = poseTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = poseTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = velocity;
    odom.twist.twist.angular.z = omega;

    //publish the message
    odom_pub.publish(odom);

    heading += omega * period;

    double dPos = velocity * period;
    x -= dPos * sin(heading);
    y += dPos * cos(heading);

    ros::spinOnce();
    rate.sleep();
  }
}