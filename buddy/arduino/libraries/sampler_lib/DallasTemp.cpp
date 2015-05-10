#include <DallasTemp.h>

DallasTemp::DallasTemp(byte* addr, byte pin, bool is_stype):
  ds(pin),
  addr(addr),
  s_type(is_stype),
  ds_state(DS_INIT),
  last_dss(0),
  temp(0),
  t(1.0/16.0,0.0),
  new_sample(false){};

void DallasTemp::sample()
{
   if (ds_state == DS_INIT)
   {
     initDS();
     ds_state = DS_READ_WAIT;
     last_dss = millis();
   }
   else
   {
     if ((millis() - last_dss) > DS_WAIT)
     {
       readDS();
       new_sample = true;
       ds_state = DS_INIT;
     }
   }
}

void DallasTemp::initDS()
{
  ds.reset();
  ds.select(addr);
  // Start conversion with parasite power
  ds.write(0x44, 1);
} 
  
void DallasTemp::readDS()
{
  ds.reset();
  ds.select(addr);    
  // Read Scratchpad
  ds.write(0xBE);         
  for (int i = 0; i < MAX_DS_LEN; ++i) {
    ds_buffer[i] = ds.read();
  }
  
  temp = int16_t(ds_buffer[1] << 8) | ds_buffer[0];
  if (s_type) {
    temp = temp << 3; // 9 bit resolution default
    if (ds_buffer[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      temp = (temp & 0xFFF0) + 12 - ds_buffer[6];
    }
  } 
  else 
  {
    byte cfg = (ds_buffer[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) temp = temp & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) temp = temp & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) temp = temp & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
}

