#include <Timer.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

///ADC publish timer
Timer t;

//Define constants
enum {
  DRIVER_NUM=4, 
  MAX_IDLE=1000,
  ADC_SAMPLE=100,
  PWM_MAX=255};
//PWM pins are selected to be dependent on timers 3,4 on the Mega2560
//Pins 2,3,5 (Timer3), Pins 6,7,8 (Timer4)
uint8_t pwm[DRIVER_NUM] = {2,3,5,6};
uint8_t dir[DRIVER_NUM] = {22,24,26,28};
uint8_t cs[DRIVER_NUM] = {A0,A1,A2,A3};
uint8_t volt = A15;

//Current sensing variables
float cs_adc[DRIVER_NUM]={0};
float cs_calib_a[DRIVER_NUM]={5.0/1023,5.0/1023,5.0/1023,5.0/1023};
float cs_calib_b[DRIVER_NUM]={0,0,0,0};

//Voltage variables
float volt_calib_a = 55.0/1023;
float volt_calib_b = 0.04;
long int last_s = 0;
float Tv = 0.5;

//Define ROS callbacks for input data
void pwmCb(const std_msgs::Float32MultiArray& values);
//Define ROS output variables
std_msgs::Float32MultiArray csout;
std_msgs::Float32 batteryout;
//Define ROS subscribers and publishers
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> pwm_in("pwm_out", pwmCb);
ros::Publisher current_out("current", &csout);
ros::Publisher battery_voltage("battery_voltage", &batteryout);

//Define internals
long int last_cmd(0);

void setup(){
  //Init data
  batteryout.data = 0;
  //Configure input/output pins
  for (int i=0; i<DRIVER_NUM; ++i)
  {
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
    pinMode(cs[i], INPUT);
    pinMode(volt, INPUT);
  }

  //Zero outputs
  zeroAll();

  //Setup ROS
  nh.initNode();
  nh.subscribe(pwm_in);
  nh.advertise(current_out);
  nh.advertise(battery_voltage);

  //Setup the csout array
  csout.data = cs_adc;
  csout.data_length = DRIVER_NUM;
  
  //Configure prescalers for PWM drivers (maximum frequency)  
  TCCR3B = TCCR3B & 0b11111000 | 0x01;
  TCCR4B = TCCR4B & 0b11111000 | 0x01;
  
  ///Setup timer 
  t.every(100, publishAdc);
}

void loop(){
  //Acquire current sensing
  currentSensing();
  //Acquire batter voltage
  voltageSensing();
  //Check for new ROS stuff
  nh.spinOnce();
  //Safety timeout
  if (millis() - last_cmd > MAX_IDLE)
  {
    zeroAll();
  }      
  //Update scheduler
  t.update();
}

void pwmCb(const std_msgs::Float32MultiArray& values)
{
  //Safety check
  if (values.data_length != DRIVER_NUM) return;

  for (int i=0; i<DRIVER_NUM; ++i)
  {
    if ((values.data[i] > 0))
    {
      digitalWrite(dir[i], HIGH);
      analogWrite(pwm[i], uint8_t(PWM_MAX*values.data[i]));
    }
    else
    {
      digitalWrite(dir[i], LOW);
      analogWrite(pwm[i], uint8_t(-PWM_MAX*values.data[i]));
    }
  }

  //Reset idle command
  last_cmd = millis();
}

void zeroAll()
{
  for (int i=0; i<DRIVER_NUM; ++i)
  {
    digitalWrite(dir[i], 0);
    analogWrite(pwm[i], 0);
  }
}

void currentSensing()
{
  for (int i=0; i<DRIVER_NUM; ++i)
  {
    //Read analog input
    cs_adc[i] = analogRead(cs[i]);
    //Scale to real value
    cs_adc[i] = cs_calib_a[i]*cs_adc[i] + cs_calib_b[i];
  }
}

void voltageSensing()
{
  //Read analog input
  int voltage = analogRead(volt);
  //Scale to real value
  float um = volt_calib_a*voltage + volt_calib_b;
  long int Ts = millis() - last_s;
  last_s = millis();
  float fc = Tv/Ts*1000;  
  
  if (Ts > 0)
    batteryout.data = (fc*batteryout.data + um)/(1+fc);
}

void publishAdc()
{
   //Return sensing data  
   current_out.publish(&csout);
   battery_voltage.publish(&batteryout);
}

