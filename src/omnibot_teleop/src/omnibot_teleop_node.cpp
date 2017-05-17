#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher velCmd;

void joy(const sensor_msgs::Joy::ConstPtr msg) {
  geometry_msgs::Twist vel;

  vel.linear.y = msg->axes[1];
  vel.angular.z = msg->axes[2];
  velCmd.publish(vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle n;

  velCmd = n.advertise<geometry_msgs::Twist>("cmd_velocity", 1);
  ros::Subscriber joySub = n.subscribe("/joy1", 1, joy);

  ros::Rate rate(200);
  while ( n.ok() ) {
    ros::spinOnce();
    rate.sleep();
  }
}