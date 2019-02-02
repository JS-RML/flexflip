#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/UInt16.h>


int sigL = 1;      //back finger
int sigR = 9;      //Blue
int i=1;

ros::NodeHandle nh;
void soft_cb(const std_msgs::UInt16& cmd_msg){
   digitalWrite(13, HIGH-digitalRead(13));  
   if(cmd_msg.data==0){
      analogWrite(sigR, 0);  
      analogWrite(sigL, 0); 
   }
   else if(cmd_msg.data==1){
      analogWrite(sigL, 140); 
      delay(500);
      analogWrite(sigR, 210);
   }
   else if(cmd_msg.data==2){
      analogWrite(sigR, 200);  
      analogWrite(sigL, 210); 
   }
}

ros::Subscriber<std_msgs::UInt16> sub("soft", soft_cb);


void setup()
{ 
  pinMode(sigL, OUTPUT); 
  pinMode(sigR, OUTPUT);
  pinMode(13,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{       
  nh.spinOnce();
  delay(1);
 
}
