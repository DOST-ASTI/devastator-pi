/*
   September 10, 2018
   The program below is a modified version of the sensor program.
   The ultrasonic sensor in front (next to the digital IR) has been added and the mobot is responding based on the
   commands from the ROS master.
*/

// Include necessary libraries
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#define USE_USBCON

// Pin assignments
#define aIR_FR A0 // Analog IR Front Right
#define aIR_FL A1 // Analog IR Front Left
#define dIR_BR 11 // Digital IR Back Right
#define dIR_BL 12 // Digital IR Back Left
#define dIR_F 10 // Digital IR Front
#define trigPin_F 2 // Ultrasonic's Trigger Pin (front)
#define echoPin_F 3 // Ultrasonic's Echo Pin (front)
#define trigPin_D 9 // Ultrasonic's Trigger Pin (down)
#define echoPin_D 8 // Ultrasonic's Echo Pin (down)
#define E1 5 // M1 Speed Control
#define E2 6 // M2 Speed Control
#define M1 4 // M1 Direction Control
#define M2 7 // M2 Direction Control

// Sets node handle name
ros::NodeHandle nh;

// Motor Directions
void stop()
{
  analogWrite(E1,LOW);
  analogWrite(E2,LOW);
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
}

void advance(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  digitalWrite(M1,HIGH);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,LOW);
}

void back_off(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  analogWrite (E2,default_speedB);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
}

void back_and_turnL(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  analogWrite (E2,default_speedB);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  delay(700);
  analogWrite (E1,default_speedA);
  digitalWrite(M1,LOW);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,LOW);
  delay(800);
}

void back_and_turnR(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  analogWrite (E2,default_speedB);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  delay(800);
  analogWrite (E1,default_speedA);
  digitalWrite(M1,HIGH);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,HIGH);
  delay(800);
}

void back_manual(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  analogWrite (E2,default_speedB);
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
}

void turn_L(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  digitalWrite(M1,LOW);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,LOW);
}

void turn_R(int default_speedA,int default_speedB)
{
  analogWrite (E1,default_speedA);
  digitalWrite(M1,HIGH);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,HIGH);
}

void turn_around(int default_speedA, int default_speedB)
{
  
  analogWrite (E1,default_speedA);
  digitalWrite(M1,HIGH);
  analogWrite (E2,default_speedB);
  digitalWrite(M2,HIGH);  
  delay(1250);
  
}

// Functions for getting sensor data and converting them to meters
float aIR_FR_Range()
{
  float distance = 0;
  float compute;
  double sample = 0.0;
  unsigned long timer;
  if ((millis() - timer) > 55)
  {
    sample = analogRead(aIR_FR);
    compute = 12343.85 * pow(sample, -1.15);
    distance = (compute) / 100;
    timer = millis();
    return distance;
  }
}

float aIR_FL_Range()
{
  float distance = 0;
  float compute;
  double sample = 0.0;
  unsigned long timer;
  if ((millis() - timer) > 55)
  {
    sample = analogRead(aIR_FL);
    compute = 12343.85 * pow(sample, -1.15);
    distance = (compute) / 100;
    timer = millis();
    return distance;
  }
}

int dIR_BR_Range()
{

  int value = digitalRead(dIR_BR);
  return value;
}

int dIR_BL_Range()
{

  int value = digitalRead(dIR_BL);
  return value;
}

int dIR_F_Range()
{
  int value = digitalRead(dIR_F);
  return value;
}

float Sonic_D_Range()
{
  long duration;
  float distance = 0;
  digitalWrite(trigPin_F, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_F, LOW);

  duration = pulseIn(echoPin_F, HIGH);

  distance = (duration / 2) * 0.00034; // Distance in meters
  return distance;
  
}

float Sonic_F_Range()
{
  long duration;
  float distance = 0;
  digitalWrite(trigPin_D, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_D, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_D, LOW);

  duration = pulseIn(echoPin_D, HIGH);

  distance = (duration / 2) * 0.00034; // Distance in meters
  return distance;
}

void assess_all(float &ir_fr, float &ir_fl, int &dIR_br, int &dIR_bl, float &sonic_d, float &sonic_f, int &dIR_f){
  ir_fr = aIR_FR_Range();
  ir_fl = aIR_FL_Range();
  dIR_br = dIR_BR_Range();
  dIR_bl = dIR_BL_Range();
  sonic_f = Sonic_F_Range();
  sonic_d = Sonic_D_Range();
  dIR_f = dIR_F_Range();
}
void assess_LR(float &ir_fr, float &ir_fl)
{
  ir_fr = aIR_FR_Range();
  ir_fl = aIR_FL_Range();
}

void assess_back(int &dIR_br, int &dIR_bl){
  dIR_br = dIR_BR_Range();
  dIR_bl = dIR_BL_Range(); 
}

void assess_front (float &sonic_f, float &sonic_d, int &dIR_f){
  sonic_f = Sonic_F_Range();
  sonic_d = Sonic_D_Range();
  dIR_f = dIR_F_Range();
}
// ROS Subscriber Callback
// This simply stores the messages coming from the Warning topic publisher.
bool automode = false;
int flag_data = 0;
void FlagCallback(const std_msgs::Int32& flag)
{
  flag_data = flag.data;
}

void OverrideCallback(const std_msgs::Bool& override_msg)
{
  automode = override_msg.data;
}

// ROS Publishers and Subscriber Instances
ros::Subscriber<std_msgs::Int32> flag_sub("nav_flag", &FlagCallback);
ros::Subscriber<std_msgs::Bool> override_sub("override_status", &OverrideCallback);

void setup()
{
  // Pin Setup for Ultrasonic and Motors
  int i;
  for (i = 4; i <= 7; i++) pinMode(i, OUTPUT);
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_F, INPUT);
  pinMode(trigPin_D, OUTPUT);
  pinMode(echoPin_D, INPUT);

  // Initialize node handle and set the baud rate to 57600
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(flag_sub);
  nh.subscribe(override_sub);

  // Sensor Message Descriptions
  // These descriptions are according to the descriptions
  // in the sensor_msgs/Range.msg documentation found here: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
  nh.spinOnce();
}

void loop()
{
  // Motor Speed Assignments 
  int default_speed = 100;
  int slow_speed = 80;

  // Get Sensor Data
  float aIR_FR_range;
  float aIR_FL_range;
  int dIR_BR_range;
  int dIR_BL_range;
  int dIR_F_range;
  float sonic_F_range;
  float sonic_D_range;

  // Range limits
  float sonic_FF_zone = 0.20;
  float sonic_FD_zone = 0.20;
  float aIR_FR_zone = 0.20;
  float aIR_FL_zone = 0.20;
  
  // Get initial sensor values
  assess_all(aIR_FR_range, aIR_FL_range, dIR_BR_range, dIR_BL_range,sonic_D_range, sonic_F_range, dIR_F_range);

//  Serial.print("aIR_R:");
//  Serial.println(aIR_FR_range);
//  Serial.print("aIR_L:");
//  Serial.println(aIR_FL_range);
  Serial.print("DIR_F: ");
  Serial.println(dIR_F_range);
  if (dIR_F_range) Serial.println("Nothing detected");
  //Serial.print("sonic_F:");
  //Serial.println(sonic_F_range);
  //Serial.print("sonic_D:");
  //Serial.println(sonic_D_range);

  //Autonavigation
  if (automode){
    assess_front(sonic_F_range, sonic_D_range, dIR_F_range);
    if (sonic_F_range > sonic_FF_zone && aIR_FR_range > aIR_FR_zone && aIR_FL_range > aIR_FL_zone){
      advance(default_speed,default_speed);
      assess_LR(aIR_FR_range, aIR_FL_range);
    }
    else if (sonic_F_range <= sonic_FF_zone && aIR_FR_range > aIR_FR_zone && aIR_FL_range > aIR_FL_zone){
      assess_back(dIR_BR_range, dIR_BL_range);
      if (dIR_BR_range && dIR_BL_range){
        back_off(slow_speed,slow_speed);
        delay(500);
        turn_R(default_speed, default_speed);
        delay(500);
        float right_distance = aIR_FR_Range();
        turn_L(default_speed, default_speed);
        float left_distance = aIR_FL_Range();
        delay(1000);
        turn_R(default_speed, default_speed);
        delay(500);
        if (right_distance > left_distance){
          turn_R(default_speed, default_speed);
          delay(500);
        }
        else if (left_distance > right_distance){
          turn_L(default_speed, default_speed);
          delay(500);
        }
        else if (aIR_FR_range < aIR_FR_zone && aIR_FL_range < aIR_FL_zone){
          turn_around(default_speed, default_speed);
        }
      }
      else
      {
        assess_LR(aIR_FR_range, aIR_FL_range);
        if (aIR_FR_range > aIR_FL_range)
          turn_R(default_speed, default_speed);
        else if (aIR_FL_range > aIR_FR_range)
          turn_L(default_speed,default_speed);
        else{
          turn_around(default_speed,default_speed);
        }
      }
      
        
    }

    else if (sonic_F_range > sonic_FF_zone && aIR_FR_range > aIR_FR_zone && aIR_FL_range <= aIR_FL_zone)
      turn_R(default_speed,default_speed);

    else if (sonic_F_range > sonic_FF_zone && aIR_FR_range <= aIR_FR_zone && aIR_FL_range > aIR_FL_zone)
      turn_L(default_speed,default_speed);
      
    else if (sonic_F_range <= sonic_FF_zone && aIR_FR_range > aIR_FR_zone && aIR_FL_range <= aIR_FL_zone){
      back_off(slow_speed,slow_speed);
      delay(400);
      turn_R(default_speed,default_speed);
    }

    else if (sonic_F_range <= sonic_FF_zone && aIR_FR_range <= aIR_FR_zone && aIR_FL_range > aIR_FL_zone){
      back_off(slow_speed,slow_speed);
      delay(400);
      turn_L(default_speed,default_speed);
    }

    else if (sonic_F_range <= sonic_FF_zone && aIR_FR_range <= aIR_FR_zone && aIR_FL_range <= aIR_FL_zone)
    {
      assess_back(dIR_BR_range, dIR_BL_range);
      if (dIR_BR_range && dIR_BL_range){
        back_off(slow_speed,slow_speed);
        delay(500);
        //assess_LR(aIR_FR_range, aIR_FL_range);
        turn_R(default_speed, default_speed);
        delay(500);
        float right_distance = aIR_FR_Range();
        turn_L(default_speed, default_speed);
        float left_distance = aIR_FL_Range();
        delay(1000);
        turn_R(default_speed, default_speed);
        delay(500);
        if (right_distance > left_distance){
          turn_R(default_speed, default_speed);
          delay(500);
        }
        else if (left_distance > right_distance){
          turn_L(default_speed, default_speed);
          delay(500);
        }
        else if (aIR_FR_range < aIR_FR_zone && aIR_FL_range < aIR_FL_zone){
          turn_around(default_speed, default_speed);
        }
      }
      
      else if (!dIR_BR_range && !dIR_BL_range)
      {
        assess_front(sonic_F_range, sonic_D_range, dIR_F_range );
        if (sonic_F_range > sonic_FF_zone||sonic_F_range <= sonic_FF_zone)
        {
          assess_LR(aIR_FR_range,aIR_FL_range);
          if (aIR_FR_range > aIR_FL_range)
            turn_R(default_speed,default_speed);
          else if (aIR_FL_range > aIR_FR_range)
            turn_L(default_speed,default_speed);
          else if (aIR_FR_range < aIR_FR_zone && aIR_FL_range < aIR_FL_zone){
            turn_around(default_speed,default_speed);
          }
        }
        else
          stop();
      }

      else if ((!dIR_BR_range && dIR_BL_range) || (dIR_BR_range && !dIR_BL_range))
      {
        assess_LR(aIR_FR_range,aIR_FL_range);
        if (aIR_FR_range > aIR_FL_range)
            turn_R(default_speed,default_speed);
          else if (aIR_FL_range > aIR_FR_range)
            turn_L(default_speed,default_speed);
          else{
            turn_around(default_speed,default_speed);
          }
      }
    }
    else{
      back_off(default_speed,default_speed);
      delay(500);
      turn_around(default_speed, default_speed);
    }
  }
  
  // Manual Navigation
  else 
  {
    if (flag_data == 1){
      advance(default_speed,default_speed);
    }
    else if (flag_data == 2){
      turn_R(default_speed,default_speed);
    }
    else if (flag_data == 3){
      turn_L(default_speed,default_speed);
    }
    else if (flag_data == 5){
      back_manual(slow_speed,slow_speed);
    }
    else stop();
  }
  nh.spinOnce();
  delay(10);
}
