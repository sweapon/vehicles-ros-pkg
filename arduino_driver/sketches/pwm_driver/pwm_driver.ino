#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

enum {diver_num=6};

int pwm[driver_num] = {2,3,4,5,6,7};
int dir[driver_num] = {22,23,24,25,26,27};

void pwm_cb(const std_msgs::Int16MultiArray& values)
{
   for (int i=0; i<6; ++i)
   {
       digitalWrite(dir[i], (values.data[i] > 0)?HIGH:LOW);
       analogWrite(pwm[i], 
       uint8_t((values.data[i] > 0)?values.data[i]:-values.data[i]));
   }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("pwm_out", pwm_cb);

void zeroAll()
{
  for (int i=0; i<6; ++i)
  {
       pinMode(dir[i], OUTPUT);
       digitalWrite(dir[i], 0);
       analogWrite(pwm[i], 0);
  }
}

void setup(){
  //Zero outputs
  zeroAll();
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}


