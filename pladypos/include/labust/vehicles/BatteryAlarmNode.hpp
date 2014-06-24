#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace labust {
  namespace vehicles {

    class BatteryAlarmNode {
    public:
      BatteryAlarmNode();
      ~BatteryAlarmNode();

    private:
      void checkVoltage(const std_msgs::Float32Ptr battery_voltage);
      ros::Subscriber battery_voltage_subscriber;
      ros::Publisher alarm_publisher;
      int threshold_;
    };

  }
}
