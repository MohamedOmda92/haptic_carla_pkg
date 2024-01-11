#include <ros.h>
#include <std_msgs/Int16.h>

int val;

ros::NodeHandle nh;

std_msgs::Int16 cmd_msg;
ros::Publisher chatter("/steer_cmd", &cmd_msg);


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  val = analogRead(A0);
  cmd_msg.data = val;
  chatter.publish( &cmd_msg );
  nh.spinOnce();
  delay(50);
}
