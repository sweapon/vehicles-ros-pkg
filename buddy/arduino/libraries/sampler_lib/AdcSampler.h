#ifndef ADCSAMPLER_H
#define ADCSAMPLER_H

///Linear sensor values
class AdcSampler
{
public:
   ///Main constructor
   AdcSampler(byte adc, float a = 1.0, float b = 0.0):
	adc(adc),t(a,b){};
   ///Main sampling function
   void sample()
   {
     data = analogRead(adc);
   }
   ///Get sample function
   float getSample()
   {
     return data*t.a + t.b;
   }

private:
   ///Linear sensor
   LinearSensor t;
   ///Sample data
   uint16_t data;
   ///Adc value
   byte adc;
};

#endif

