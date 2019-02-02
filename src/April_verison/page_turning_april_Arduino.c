#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Empty.h>


int sigL = 5;      //Yellow
int sigR = 6;      //Blue
int i=1;
ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));  
  // blink the led
   if(i==1){
      analogWrite(sigL, 125*i);  // set the pressure of finger by pwm
      delay(500);
      analogWrite(sigR, 250*i); 
      //delay(2000) ;
      //analogWrite(sigL, 0);  // set the pressure of finger by pwm
      //delay(1000);
      //analogWrite(sigR, 0); 
      //delay(2000) ;
   }
   else{
      analogWrite(sigR, 250*i);  // set the pressure of finger by pwm
      delay(1000);
      analogWrite(sigL, 125*i); 
   }
   if(i==1){i=0;}
   else if (i==0){i=1;}
   
}

ros::Subscriber<std_msgs::Empty> sub("soft", &messageCb );

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
