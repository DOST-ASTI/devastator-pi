#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


class Override
{
public:
	Override();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void keyCallback(const std_msgs::String::ConstPtr& key_msg);
  void Publisher();
	
private:
	ros::NodeHandle nh_;
	ros::Publisher override_pub;
  ros::Subscriber joy_sub_;
  ros::Subscriber key;
  bool currentReading;
	bool lastReading;
  bool flag;
  std_msgs::String override_key;
  std_msgs::String override_;
  std_msgs::String reset_;
  std_msgs::Float32 button_status;
  std_msgs::Bool override_status;
};


Override::Override()
{

  override_pub = nh_.advertise<std_msgs::Bool>("override_status", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Override::joyCallback, this);
  key = nh_.subscribe<std_msgs::String>("key", 1, &Override::keyCallback, this);
}

void Override::keyCallback(const std_msgs::String::ConstPtr& key_msg)
{
  override_.data = 'f'; // Pressing 'f' switches to manual mode
  reset_.data = 'r'; // Pressing 'r' switches back to auto mode
  override_key.data = key_msg->data;
  if (override_key.data == override_.data){
    override_status.data = true;
    ROS_INFO("AUTONAVIGATION");
  }
  else if (override_key.data == reset_.data){
    override_status.data = false;
    ROS_INFO("MANUAL MODE");
  }
}

void Override::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   button_status.data=joy->buttons[4];
  if (button_status.data == 0.0){
    currentReading = false;
  }else currentReading = true;

  if (currentReading && !lastReading) {
    flag=!flag;
    if (flag) {
      override_status.data = true;
      ROS_INFO("MANUAL OVERRIDE");
    }
    else {
      override_status.data = false;
      ROS_INFO("AUTONAVIGATION MODE");
    }
  }
  lastReading = currentReading;
}

void Override::Publisher()
{
  override_pub.publish(override_status);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "override_node");
  ROS_INFO("Node Started...");
  Override override;
  ros::Rate r(10);
  while (ros::ok()){
    ros::spinOnce();
    override.Publisher(); 
    r.sleep();  
  }
  return 0;
}