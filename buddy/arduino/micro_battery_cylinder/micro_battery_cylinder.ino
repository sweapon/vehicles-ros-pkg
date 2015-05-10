#include <Timer.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <DallasTemp.h>
#include <AdcSampler.h>
#include <SerialHandler.h>
#include <BuddyMsgs.h>

///ADC publish timer
enum {PUB_TIME=250};
Timer t;
/// Sensor connections
// Temperature one wire connected to pin 2 (2.2K pullup)
// Address of the temperature sensor
byte addr[8]={0x28, 0xFF, 0xF9, 0x3E, 0x4E, 0x04, 0x00, 0xBE};
DallasTemp dstemp(addr,2,false);
// Leak sensor
AdcSampler leak(A0, 5.015/1024);
// Pressure sensor
AdcSampler pressure(A5, 5.015/1024);

//External connection to vision cylinder (RX=10, TX=11)
SoftwareSerial swSerial(11, 10);
//Serial handler for the master cylinder side
void onMaster(byte* payload, byte len, byte sender, byte msg_type);
SerialHandler master(onMaster);
//Serial handler for the vision cylinder side
void onVision(byte* payload, byte len, byte sender, byte msg_type);
SerialHandler vision(onVision);

void sendMeas()
{
   float data[LPTData::ELEM]=
   {leak.getSample(), pressure.getSample()};
   byte* buf = reinterpret_cast<byte*>(&data);
   byte type = LPData::MSG_TYPE;
   byte len = LPData::LEN;
   //Forward sensing data
   if (dstemp.hasNew())
   {
     //Calculate data
     data[LPTData::TEMP]=dstemp.getSample();
     type = LPTData::MSG_TYPE;
     len = LPTData::LEN;
   }
   
   master.writeSerial(&Serial1, buf, len, BATTERY_ID, type);
   //vision.writeSerial(&swSerial, buf, len, BATTERY_ID, type);   
}

void onMaster(byte* payload, byte len, byte sender, byte msg_type)
{
  //Forward master message to cylinder
  //vision.writeSerial(&swSerial, payload, len, sender, msg_type);
}

void onVision(byte* payload, byte len, byte sender, byte msg_type)
{
  //Forward vision cylinder data to master
  master.writeSerial(&Serial1, payload, len, sender, msg_type);
}

void setup(void) {
  //Set pin modes

  //Serial connection to Arduino Uno in Master cylinder
  Serial1.begin(INTERCOMMS_BAUD);
  while (!Serial1);
  
  //Serial connection to Arduino Micro in Vision cylinder
  swSerial.begin(INTERCOMMS_BAUD);
  ///Setup timer 
  t.every(PUB_TIME, sendMeas);
}

void loop(void) {
  //Sample analogs
  dstemp.sample();
  leak.sample();
  pressure.sample();
  
  //Read serial ports 
  vision.readSerial(&swSerial);
  //master.readSerial(&Serial1);
  
  //Update scheduler
  t.update();
}


