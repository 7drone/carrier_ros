#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "k_means.hpp"

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> ranges = scan->ranges;

    // K-Means 알고리즘을 사용하여 군집화
    int num_clusters = 5;
    KMeans kmeans(num_clusters);
    std::vector<int> labels = kmeans.fit(ranges);

    // 군집화 결과 출력
    ROS_INFO("Cluster Labels: ");
    for (int i = 0; i < labels.size(); i++) {
        ROS_INFO_STREAM(labels[i]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_kmeans");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);

    ros::spin();

    return 0;
}
