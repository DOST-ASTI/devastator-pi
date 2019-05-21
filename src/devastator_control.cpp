//  Devastator bot control module
//  October 23, 2018
//  DOST-ASTI, RDD Team
//***************************************************
//-----------Robot Control Flags---------------------
// 1 - Move Forward
// 2 - Turn Right
// 3 - Turn Left
// 4 - Turn around (Rotate Right)
// 5 - Move Backward
// 6 - Stop
/////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class Devastator {
public:
  Devastator();
  //Create a public class member to publish flags used for robot movement.
  void FlagPublisher(int x);

  //Override and geometry variables.
  bool override_flag();
  bool override_;
  float* vel_data();


private:
  //Override Flags and geometry data subscribers.
  void VelCallback(const geometry_msgs::Twist::ConstPtr& twist);
  void OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg);

  //ROS variables
  ros::NodeHandle nh_;
  ros::Publisher nav_flag_pub;
  ros::Publisher buzz_pub;
  ros::Subscriber vel_sub;
  ros::Subscriber override_sub;

  //Set variables for sensor data
  float linear_,angular_;
};


Devastator::Devastator()
{
  nav_flag_pub = nh_.advertise<std_msgs::Int32>("nav_flag", 1);
  buzz_pub = nh_.advertise<std_msgs::Bool>("buzz_flag", 1);
  override_sub = nh_.subscribe<std_msgs::Bool>("override_status", 10, &Devastator::OverrideCallback, this);
  vel_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Devastator::VelCallback, this);
}


void Devastator::OverrideCallback(const std_msgs::Bool::ConstPtr& override_msg)
{
  override_ = override_msg->data;
}

//Get geometry data for robot displacement values from ROS geometry publisher.
void Devastator::VelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  linear_ = vel_msg->linear.x;
  angular_ = vel_msg->angular.z;
}


//Get Functions
bool Devastator::override_flag()
{
  return override_;
}

float* Devastator::vel_data()
{
  float* vel = new float[2];
  vel[0] = linear_;
  vel[1] = angular_;
  return vel;
}

void Devastator::FlagPublisher(int x)
{
  std_msgs::Int32 flag_data;
  flag_data.data = x;
  nav_flag_pub.publish(flag_data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "devastator_control");
  ROS_INFO("Node Started...");
  Devastator devastator;
  ros::Rate r(10);
  while (ros::ok()){
    //Calls all ROS callback functions.
    ros::spinOnce();

    //If no override is detected. Navigate autonomously.
    float* vel = devastator.vel_data();

    if (vel[0] < 0.0) {
      devastator.FlagPublisher(5);
    }else if (vel[0] > 0.0) {
      devastator.FlagPublisher(1);
    }else if (vel[1] < 0.0) {
      devastator.FlagPublisher(2);
    }else if (vel[1] > 0.0) {
      devastator.FlagPublisher(3);
    }else if (vel[0] == 0.0 && vel[1] == 0.0){
      devastator.FlagPublisher(6);
    }else
      devastator.FlagPublisher(6);
    r.sleep();
  }
  return 0;
}
