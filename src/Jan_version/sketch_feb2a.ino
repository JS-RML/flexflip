#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/UInt16.h>


int sigL = 5;      //back finger
int sigR = 9;      //Blue
int i=1;

ros::NodeHandle nh;
void soft_cb(const std_msgs::UInt16& cmd_msg){
   digitalWrite(13, HIGH-digitalRead(13)); 
   for(int kk=0; kk<=255;i++){ 
     if(cmd_msg.data==kk){
      analogWrite(sigL, kk); 
      delay(100);
      analogWrite(sigR, 180);
    }
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
