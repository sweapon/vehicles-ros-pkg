#ifndef DALLAS_TEMP_H
#define DALLAS_TEMP_H
#include <OneWire.h>
#include <LinearSensor.h>

///Dallas temperature sensor sampler
class DallasTemp
{
  enum {MAX_DS_LEN=9, DS_WAIT=650};
  enum {DS_INIT=0, DS_READ_WAIT};
 public:
   ///Generic constructor
   DallasTemp(byte* addr, byte pin, bool is_stype = true);
  
   ///Main sampling function
   void sample();
   ///Return the sampled value
   float getSample()
   { 
     new_sample=false;
     return t.a*temp + t.b;
   }
   ///Test if new sample is ready
   bool hasNew(){return new_sample;};
   ///Sets the linear sensor values
   void setSensor(float a, float b){t.a = a;t.b = b;}

 private:
   ///Init parasitic device
   void initDS();
   ///Read the device
   void readDS();
   ///The one-wire protocol
   OneWire ds;
   ///The device address
   byte* addr;
   ///Device type
   bool s_type;
   ///Buffer of the data
   byte ds_buffer[MAX_DS_LEN];
   ///Interogation state
   byte ds_state;
   ///Last turn-on time
   long int last_dss; 
   ///Last sample
   int16_t temp;
   ///New sample flag
   bool new_sample;
   ///Linear sensor values
   LinearSensor t;
};






#endif

