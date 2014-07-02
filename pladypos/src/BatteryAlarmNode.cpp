#include <labust/vehicles/BatteryAlarmNode.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sound_play/sound_play.h>

using namespace labust::vehicles;

BatteryAlarmNode::BatteryAlarmNode() :
    threshold_(18),
    r(0.2) {
  ros::NodeHandle ph("~");
  ph.setParam("threshold", threshold_);
  battery_voltage_subscriber = nh.subscribe("battery_voltage", 1, &BatteryAlarmNode::checkVoltage, this);
  alarm_publisher = nh.advertise<std_msgs::Bool>("battery_alarm", 1);
}

BatteryAlarmNode::~BatteryAlarmNode() {}

void BatteryAlarmNode::checkVoltage(const std_msgs::Float32::ConstPtr& battery_voltage) {
  if ( battery_voltage->data < threshold_) {
    std_msgs::Bool alarm;
    alarm.data = true;
    alarm_publisher.publish(alarm);
    ROS_ERROR("Battery voltage below threshold!");
    sound_play::SoundClient sound_client;
    r.sleep();
    sound_client.startWave("/home/ivor/battery_alarm.wav");
    r.sleep();
    r.sleep();
    sound_client.say("Shutdown, battery low, self destruct. Danger! Danger!");
    r.sleep();
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "battery_alarm_node");
  BatteryAlarmNode bat_alarm;
  ros::spin();
}
