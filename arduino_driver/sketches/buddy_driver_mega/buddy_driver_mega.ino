#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <OneWire.h>

//Define pin configuration
enum {
  driver_num=6, max_idle=1000, one_wire_pin=30, temp_cycle=120, temp_par=0, temp_size =9};
//PWM pins are selected to be dependent on timers 3,4 on the Mega2560
//Pins 2,3,5 (Timer3), Pins 6,7,8 (Timer4)
uint8_t pwm[driver_num] = {
  2,3,5,6,7,8};
uint8_t dir[driver_num] = {
  22,23,24,25,26,27};
uint8_t cs[driver_num] = {
  A0,A1,A2,A3,A4,A5};

//Current sensing data
float cs_adc[driver_num]={
  0};
float cs_calib_a[driver_num]={
  5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023};
float cs_calib_b[driver_num]={
  0,0,0,0,0,0};
//Temperature sensing
OneWire ds(one_wire_pin);
uint8_t sensor_addr[driver_num][8]=
{
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
  ,
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
  ,
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
  ,
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
  ,
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
  ,
  {
    0x10,0x7D,0x2C,0x72,0x01,0x08,0x00,0x82  }
};
float temp[driver_num]={
  0};
float temp_calib_a[driver_num]={
  5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023};
float temp_calib_b[driver_num]={
  0,0,0,0,0,0};
uint8_t tempIdx(0);
int tempCnt(0);
uint8_t temp_data[temp_size]={
  0};
//Temperature states 
#define START_TEMP 0
#define WAIT_TEMP 1 
uint8_t tempState(START_TEMP);


//Define ROS callbacks and output data
void pwm_cb(const std_msgs::Int16MultiArray& values);
void scaler_cb(const std_msgs::UInt8& prescaler);
std_msgs::Float32MultiArray csout;
std_msgs::Float32MultiArray tempout;
//Define ROS subscribers and publishers
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray> pwmIn("pwm_out", pwm_cb);
ros::Subscriber<std_msgs::UInt8> scalerIn("pwm_scaler", scaler_cb);
ros::Publisher currentSensingOut("current", &csout);
ros::Publisher temperatureOut("temp", &tempout);

//Define internals
int idle(0);

void setup(){
  //Configure input/output pins
  for (int i=0; i<driver_num; ++i)
  {
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
    pinMode(cs[i], INPUT);
  }

  //Zero outputs
  zeroAll();

  //Setup ROS
  nh.initNode();
  nh.subscribe(pwmIn);
  nh.subscribe(scalerIn);
  nh.advertise(currentSensingOut);
  nh.advertise(temperatureOut);

  //Setup the csout array
  //csout.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
  //csout.layout.dim_length  = 1;
  csout.data = cs_adc;
  csout.data_length = driver_num;

  //Setup temp array
  //csout.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension));
  //csout.layout.dim_length  = 1;
  tempout.data = temp;
  tempout.data_length = driver_num;
}

void loop(){
  //Acquire current sensing
  currentSensing();
  //Acquire temperature
  temperatureSensing();
  //Check for new ROS stuff
  nh.spinOnce();
  //Safety timeout
  if (++idle > max_idle)
  {
    zeroAll();
    idle = 0;
  }      
  //Small NOP
  delay(1);
}

void pwm_cb(const std_msgs::Int16MultiArray& values)
{
  //Safety check
  if (values.data_length != driver_num) return;

  for (int i=0; i<driver_num; ++i)
  {
    if ((values.data[i] > 0))
    {
      digitalWrite(dir[i], HIGH);
      analogWrite(pwm[i], uint8_t(values.data[i]));
    }
    else
    {
      digitalWrite(dir[i], LOW);
      analogWrite(pwm[i], uint8_t(-values.data[i]));
    }
  }

  //Return sensing data  
  currentSensingOut.publish(&csout);
  temperatureOut.publish(&tempout);

  //Reset idle command
  idle = 0;
}

void scaler_cb(const std_msgs::UInt8& prescaler)
{
  if (prescaler.data >1) 
    switch (prescaler.data)
    {
    case 1:
      TCCR3B = TCCR3B & 0b11111000 | 0x01;
      TCCR4B = TCCR4B & 0b11111000 | 0x01;
    }
}

void zeroAll()
{
  for (int i=0; i<driver_num; ++i)
  {
    digitalWrite(dir[i], 0);
    analogWrite(pwm[i], 0);
  }
}

void currentSensing()
{
  for (int i=0; i<driver_num; ++i)
  {
    //Read analog input
    cs_adc[i] = analogRead(cs[i]);
    //Scale to real value
    cs_adc[i] = cs_calib_a[i]*cs_adc[i] + cs_calib_b[i];
  }

  //currentSensingOut.publish(&csout);
}

void temperatureSensing()
{
  //temp[tempIdx] = tempCnt; //Cycle debugging
  
  if (tempState == START_TEMP)
  {
    ds.reset();
    ds.select(sensor_addr[tempIdx]);
    //Start conversion
    ds.write(0x44, temp_par);
    tempState = WAIT_TEMP;
    tempCnt = 0;
  }
  else if (++tempCnt > temp_cycle)
  {
    //Read measurement
    if (ds.reset())
    {
      ds.select(sensor_addr[tempIdx]);    
      ds.write(0xBE);
      //Read data
      ds.read_bytes(temp_data, temp_size);
      convertTemp();
    }
    else
    {
      temp[tempIdx] = -100;
    }
    
    tempIdx = ++tempIdx % driver_num;
    tempState = START_TEMP;
  }

}

void convertTemp()
{
  //Check data integrity
  if (OneWire::crc8(temp_data, 8) == temp_data[temp_size-1])
  {
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (temp_data[1] << 8) | temp_data[0];
    if (sensor_addr[tempIdx][0] == 0x10) 
    {
      raw = raw << 3; // 9 bit resolution default
      if (temp_data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - temp_data[6];
      }
    } 
    else {
      byte cfg = (temp_data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    temp[tempIdx] = (float)raw / 16.0;
  }
  else
  {
    temp[tempIdx] = -101;
  } 
}



