#include <Timer.h>
#include <OneWire.h>
#include <DallasTemp.h>
#include <BuddyMsgs.h>
#include <AdcSampler.h>
#include <SerialHandler.h>
#include <SoftwareSerial.h>
#include <std_msgs/Float32.h>
#include <ros.h>

///ADC publish timer
enum {PUB_RATE=1000, ROS_BAUD=115200};
Timer t;
/// Sensor connections
// Temperature one wire connected to pin 2 (2.2K pullup)  
// Address of the temperature sensor
byte addr[8]={0x10, 0x9C, 0x91, 0xB4, 0x02, 0x08, 0x00, 0xEB};
DallasTemp dstemp(addr,2,true);
// Leak sensor
AdcSampler leak(A0, 5.015/1024);

//External connection to battery cylinder (RX=10, TX=11)
SoftwareSerial swSerial(11, 10); 
//Serial handler for the master cylinder side
void onBattery(byte* payload, byte len, byte sender, byte msg_type);
SerialHandler battery(onBattery);

/// ROS configuration
// Define ROS output variables
std_msgs::Float32 tempout;
std_msgs::Float32 leakout;

bool new_bat_temp=false;
std_msgs::Float32 bat_tempout;
std_msgs::Float32 bat_leakout;
std_msgs::Float32 pressureout;

bool new_vis_temp=false;
std_msgs::Float32 vis_tempout;
std_msgs::Float32 vis_leakout;

// Define ROS subscribers and publishers
ros::NodeHandle nh;
ros::Publisher temp_pub("master/temperature", &tempout); 
ros::Publisher leak_pub("master/leak", &leakout);
ros::Publisher bat_temp_pub("battery/temperature", &bat_tempout); 
ros::Publisher bat_leak_pub("battery/leak", &bat_leakout);
ros::Publisher pressure_pub("pressure", &pressureout);
ros::Publisher vis_temp_pub("vision/temperature", &bat_tempout); 
ros::Publisher vis_leak_pub("vision/leak", &bat_leakout);

void onBattery(byte* payload, byte len, byte sender, byte msg_type)
{
  if ((sender != BATTERY_ID) && (sender != VISION_ID)) return;
  
  float* p = reinterpret_cast<float*>(payload);
  switch (msg_type)
  {
    case LData::MSG_TYPE:
      if (sender == BATTERY_ID)
      {
        bat_leakout.data = p[LData::LEAK];
        bat_leak_pub.publish(&bat_leakout);
      }
      else
      {
        vis_leakout.data = p[LData::LEAK];
        vis_leak_pub.publish(&vis_leakout);
      }
      break;
    case LPTData::MSG_TYPE:
        bat_tempout.data = p[LPTData::TEMP];
        new_bat_temp = true;
        bat_temp_pub.publish(&bat_tempout);
    case LPData::MSG_TYPE:
        bat_leakout.data = p[LPData::LEAK];
        pressureout.data = p[LPData::PRESSURE];
        bat_leak_pub.publish(&bat_leakout);
        pressure_pub.publish(&pressureout);
        break;
    case LTData::MSG_TYPE:
       if (sender == BATTERY_ID)
       {
        bat_leakout.data = p[LTData::LEAK];
        new_bat_temp = true;
        bat_tempout.data = p[LTData::TEMP];
        bat_temp_pub.publish(&bat_tempout);
        bat_leak_pub.publish(&bat_leakout);
       }
      else
      {
        vis_leakout.data = p[LTData::LEAK];
        new_vis_temp = true;
        vis_tempout.data = p[LTData::TEMP];
        vis_leak_pub.publish(&vis_leakout);
        vis_temp_pub.publish(&vis_tempout);
      }
      break;
    default:
      break;
  }
}

void publishAdc()
{
   //Return sensing data  
   leakout.data = leak.getSample();
   leak_pub.publish(&leakout);
   
   //Send master data to battery cylinder
   //float data[LTData::ELEM]={leak.getSample()};
   //byte len = LData::LEN;
   //byte type = LData::MSG_TYPE;
   
   if (dstemp.hasNew())
   {
     tempout.data = dstemp.getSample();
     //data[LTData::TEMP] = tempout.data;
     //len = LTData::LEN;
     //type = LTData::MSG_TYPE;
     temp_pub.publish(&tempout);
   }
   //byte* buf = reinterpret_cast<byte*>(&data);
   //battery.writeSerial(&swSerial, buf, len, MASTER_ID, type);
   
   /*td_msgs/Float32.h>vis_leak_pub.publish(&vis_leakout);
   bat_leak_pub.publish(&bat_leakout);
   if (new_vis_temp)
   {
     vis_temp_pub.publish(&vis_tempout);
     new_vis_temp = false;
   }
   
   if (new_bat_temp)
   {
     bat_temp_pub.publish(&bat_tempout);
     new_bat_temp = false;
   }*/
}

void setup(void) {
  //Set pin modes
  
  nh.getHardware()->setBaud(ROS_BAUD);
  
  //Start ROS
  nh.initNode();
  nh.advertise(temp_pub);
  nh.advertise(leak_pub);
  nh.advertise(bat_temp_pub);
  nh.advertise(bat_leak_pub);
  nh.advertise(pressure_pub);
  nh.advertise(vis_temp_pub);
  nh.advertise(vis_leak_pub);
  
  //Initialize external connection serial ports
  swSerial.begin(INTERCOMMS_BAUD);
  ///Setup timer 
  t.every(PUB_RATE, publishAdc);
}

void loop(void) {
  //Sample analogs
  dstemp.sample();
  leak.sample();
  
  //Read battery cylinder arduino micro 
  battery.readSerial(&swSerial);
  
  //Sping ROS
  nh.spinOnce();
  //Update scheduler
  t.update();
}


