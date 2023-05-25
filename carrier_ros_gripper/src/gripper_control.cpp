#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

class Control {
public:
    Control();

private:
    bool gripper(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    ros::NodeHandle n;

    ros::ServiceServer service_;
    ros::Publisher vel_pub_;
};

Control::Control() {
    service_ = n.advertiseService("/gripper", &Control::gripper, this);
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/dynamixel/cmd_vel", 10);
}

bool Control::gripper(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    geometry_msgs::Twist twist;

    ros::Rate loop_rate(10); 
    int count = 0;

    if (req.data == true) {             // gripper open
        std::cout << "Gripper Open\n";
        
        while (count < 30) {            // 수정 필요  
            twist.linear.x = -0.1;      // 방향 확인 필요

            vel_pub_.publish(twist);
            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
    } else if (req.data == false) {     // gripper close
        std::cout << "Gripper Close\n";
        
        while (count < 30) {            // 수정 필요
            twist.linear.x = 0.1;      // 방향 확인 필요

            vel_pub_.publish(twist);
            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
    }

    return res.success = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_control");
    Control dynamixel_cnt;
    ros::spin();

    return 0;
}