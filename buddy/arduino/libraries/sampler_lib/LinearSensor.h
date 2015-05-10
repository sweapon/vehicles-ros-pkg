#ifndef LINEARSENSOR_H
#define LINEARSENSOR_H

///Linear sensor values
struct LinearSensor
{
   LinearSensor():a(1.0), b(0.0){};
   LinearSensor(float a, float b):a(a),b(b){};
   ///Scaling factor
   float a;
   ///Offset factor
   float b;
};

#endif

