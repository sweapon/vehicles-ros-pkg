#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

//Define constants
enum {
  DRIVER_NUM=4, 
  MAX_IDLE=1000,
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
float volt_calib_a = 5.0/1023;
float volt_calib_b = 0;

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
int idle(0);

void setup(){
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
}

void loop(){
  //Acquire current sensing
  currentSensing();
  //Acquire batter voltage
  voltageSensing();
  //Check for new ROS stuff
  nh.spinOnce();
  //Safety timeout
  if (++idle > MAX_IDLE)
  {
    zeroAll();
    idle = 0;
  }      
  //Small NOP
  delay(1);
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

  //Return sensing data  
  current_out.publish(&csout);
  battery_voltage.publish(&batteryout);

  //Reset idle command
  idle = 0;
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
  batteryout.data = volt_calib_a*voltage + volt_calib_b;
}


