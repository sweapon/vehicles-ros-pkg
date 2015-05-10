#include <Timer.h>
#include <OneWire.h>
#include <DallasTemp.h>
#include <BuddyMsgs.h>
#include <AdcSampler.h>
#include <SerialHandler.h>
#include <SoftwareSerial.h>
#include <std_msgs/Float32.h>

#define USE_USBCON
#include <ros.h>

///ADC publish timer
enum {PUB_RATE=1000, ROS_BAUD=115200};
Timer t;
/// Sensor connections
// Temperature one wire connected to pin 2 (2.2K pullup)  
// Address of the temperature sensor
byte addr[8]={0x10, 0x32, 0x8E, 0xB4, 0x02, 0x08, 0x00, 0xB7}; 
DallasTemp dstemp(addr,2,true);
// Leak sensor
AdcSampler leak(A0, 5.015/1024);

//External connection to battery cylinder (RX=10, TX=11)
SoftwareSerial swSerial(10, 11); 
//Serial handler for the master cylinder side
void onBattery(byte* payload, byte len, byte sender, byte msg_type);
SerialHandler battery(onBattery);

/// ROS configuration
// Define ROS output variables
std_msgs::Float32 tempout;
std_msgs::Float32 leakout;

std_msgs::Float32 bat_tempout;
std_msgs::Float32 bat_leakout;
std_msgs::Float32 pressureout;

std_msgs::Float32 mas_tempout;
std_msgs::Float32 mas_leakout;

// Define ROS subscribers and publishers
ros::NodeHandle nh;
ros::Publisher temp_pub("vision/temperature", &tempout); 
ros::Publisher leak_pub("vision/leak", &leakout);
ros::Publisher bat_temp_pub("battery/temperature", &bat_tempout); 
ros::Publisher bat_leak_pub("battery/leak", &bat_leakout);
ros::Publisher pressure_pub("pressure", &pressureout);
ros::Publisher mas_temp_pub("master/temperature", &bat_tempout); 
ros::Publisher mas_leak_pub("master/leak", &bat_leakout);

void onBattery(byte* payload, byte len, byte sender, byte msg_type)
{
  if ((sender != BATTERY_ID) && (sender != MASTER_ID)) return;
  
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
        mas_leakout.data = p[LData::LEAK];
        mas_leak_pub.publish(&mas_leakout);
      }
      break;
    case LPTData::MSG_TYPE:
        bat_tempout.data = p[LPTData::TEMP];
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
        bat_tempout.data = p[LTData::TEMP];
        bat_temp_pub.publish(&bat_tempout);
        bat_leak_pub.publish(&bat_leakout);
       }
      else
      {
        mas_leakout.data = p[LTData::LEAK];
        mas_tempout.data = p[LTData::TEMP];
        mas_leak_pub.publish(&mas_leakout);
        mas_temp_pub.publish(&mas_tempout);
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
   float data[LTData::ELEM]={leak.getSample()};
   byte len = LData::LEN;
   byte type = LData::MSG_TYPE;
   
   if (dstemp.hasNew())
   {
     tempout.data = dstemp.getSample();
     data[LTData::TEMP] = tempout.data;
     len = LTData::LEN;
     type = LTData::MSG_TYPE;
     temp_pub.publish(&tempout);
   }
   byte* buf = reinterpret_cast<byte*>(&data);
   battery.writeSerial(&swSerial, buf, len, VISION_ID, type);   
}

void setup(void) {
  //Set pin modes
  
  nh.getHardware()->setBaud(ROS_BAUD);
  
  //Start ROS
  nh.initNode();
  //while(!Serial1);
  nh.advertise(temp_pub);
  nh.advertise(leak_pub);
  nh.advertise(bat_temp_pub);
  nh.advertise(bat_leak_pub);
  nh.advertise(pressure_pub);
  nh.advertise(mas_temp_pub);
  nh.advertise(mas_leak_pub);
  
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
  //battery.readSerial(&swSerial);
  
  //Sping ROS
  nh.spinOnce();
  //Update scheduler
  t.update();
}


