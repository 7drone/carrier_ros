#include <ros/ros.h>
#include <carrier_ros_msg/BatteryOne.h>
#include <carrier_ros_msg/BatteryTwo.h>

class Battery {
public:
    Battery();

    double robot_battery = 0.0, drone_battery = 0.0;

private:
    void robotBatteryCallback(const carrier_ros_msg::BatteryOne::ConstPtr& battery);
    void droneBatteryCallback(const carrier_ros_msg::BatteryOne::ConstPtr& battery);

    ros::NodeHandle n;

    ros::Publisher battery_pub_;
    ros::Subscriber robot_battery_sub_, drone_battery_sub_;
};

Battery::Battery() {
    battery_pub_ = n.advertise<carrier_ros_msg::BatteryTwo>("/battery", 10);
    robot_battery_sub_ = n.subscribe<carrier_ros_msg::BatteryOne>("/battery/robot", 10, &Battery::robotBatteryCallback, this);
    drone_battery_sub_ = n.subscribe<carrier_ros_msg::BatteryOne>("/battery/drone", 10, &Battery::droneBatteryCallback, this);
}

void Battery::robotBatteryCallback(const carrier_ros_msg::BatteryOne::ConstPtr& battery) {
    robot_battery = battery->battery;

    carrier_ros_msg::BatteryTwo battery_msg;
    battery_msg.robot_battery = robot_battery;
    battery_msg.drone_battery = drone_battery;
    battery_pub_.publish(battery_msg);
}

void Battery::droneBatteryCallback(const carrier_ros_msg::BatteryOne::ConstPtr& battery) {
    drone_battery = battery->battery;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_node");
    Battery robot_battery;

    ros::spin();
}
