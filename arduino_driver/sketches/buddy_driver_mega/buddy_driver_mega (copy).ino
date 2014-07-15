  #include <ros.h>
  #include <std_msgs/Int16MultiArray.h>
  #include <std_msgs/UInt8.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <OneWire.h>
  
  //Define pin configuration
  enum {driver_num=6, max_idle=1000, one_wire_pin=30};
  //PWM pins are selected to be dependent on timers 3,4 on the Mega2560
  //Pins 2,3,5 (Timer3), Pins 6,7,8 (Timer4)
  uint8_t pwm[driver_num] = {2,3,5,6,7,8};
  uint8_t dir[driver_num] = {22,23,24,25,26,27};
  uint8_t cs[driver_num] = {A0,A1,A2,A3,A4,A5};
  
  //Current sensing data
 float cs_adc[driver_num];
 float cs_calib_a[driver_num]={5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023};
 float cs_calib_b[driver_num]={0,0,0,0,0,0};
 //Temperature sensing
 OneWire ds(one_wire_pin);
 float temp[driver_num];
 float temp_calib_a[driver_num]={5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023,5.0/1023};
 float temp_calib_b[driver_num]={0,0,0,0,0,0};
  
  
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
    //Acquire and publish current sensing
    currentSensing();
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
     
     //Return current sensing data  
     currentSensingOut.publish(&csout);
     
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
  
  

