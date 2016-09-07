#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "simpleArDriver");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate rate(10);
  geometry_msgs::Twist msg;
  bool stop = true;
  while(ros::ok()) {
    if (stop) {
      msg.linear.x = 1.0;
      ROS_INFO("Pubing MSG! 1");
    } else {
      msg.linear.x = 0.0;
      ROS_INFO("Pubing MSG! 0");
    }
    stop = !stop;
    pub.publish(msg);
    rate.sleep();
  }
}
