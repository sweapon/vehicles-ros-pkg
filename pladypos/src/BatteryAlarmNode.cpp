#include <labust/vehicles/BatteryAlarmNode.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace labust::vehicles;

BatteryAlarmNode::BatteryAlarmNode() :
    threshold_(18) {
  ros::NodeHandle nh;
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
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "battery_alarm_node");
  BatteryAlarmNode bat_alarm;
  ros::spin();
}
