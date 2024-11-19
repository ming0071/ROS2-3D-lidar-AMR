#include "ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

int pwm1 =0;
int pwm2 =0;

void velA(const std_msgs::Float64 &msg)
{
  if(msg.data>0)
  {
    digitalWrite(44,LOW);
    digitalWrite(45,LOW);
  }
  else if(msg.data<0)
  {
    digitalWrite(44,HIGH);
    digitalWrite(45,LOW);
  }
  else
  {
    digitalWrite(45,HIGH);
  }
}

void velB(const std_msgs::Float64 &msg)
{
  if(msg.data>0)
  {
    digitalWrite(34,HIGH);   
    digitalWrite(35,LOW);
  }
  else if(msg.data<0)
  {
    digitalWrite(34,LOW);
    digitalWrite(35,LOW);
  }
  else
  {
    digitalWrite(35,HIGH);
  }
}
void pwmA(const std_msgs::Float64 &msg)
{
  pwm1 = msg.data;
  analogWrite(8,pwm1);
}
void pwmB(const std_msgs::Float64 &msg)
{
  pwm2 = msg.data;
  analogWrite(9,pwm2);
}

ros::Subscriber<std_msgs::Float64> sub1("/pwmA", &pwmA);
ros::Subscriber<std_msgs::Float64> sub2("/pwmB", &pwmB);
ros::Subscriber<std_msgs::Float64> sub3("/velA", &velA);
ros::Subscriber<std_msgs::Float64> sub4("/velB", &velB);

void setup()
{
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  
  pinMode(8, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(35, OUTPUT);
}

void loop() 
{
  nh.spinOnce();
  delay(30);
}
