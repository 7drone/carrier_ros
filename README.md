# Carrier

Separable Integrated Drone-Mobile Robot System for Unmanned Patrol.

Carrier is a ROS-based autonomous robot system that combines a mobile robot and a drone. The mobile robot transports and wirelessly charges the drone, then deploys it for last-mile patrol or scouting in areas that are difficult for the ground robot to reach.

## Project Page

https://7drone.github.io/carrier_ros/

## Overview

The system integrates:

- Mobile robot localization using wheel odometry, IMU, and RTK GNSS with EKF
- Environment perception using RealSense cameras and LiDAR
- Obstacle detection with depth-camera and YOLO-based perception
- Naver map conversion for global planning
- Wireless drone charging on the mobile robot
- ArUco-marker-based drone docking

## Resources

- [Project Page](https://7drone.github.io/carrier_ros/)
- [Paper](https://7drone.github.io/carrier_ros/assets/carrier-icros-2023.pdf)
- [Demo Video](https://youtu.be/49tMCc3HBR0)

## Citation

```bibtex
@inproceedings{kim2023carrier,
  title = {Separable Integrated Drone-Mobile Robot System for Unmanned Patrol},
  author = {Kim, Jeonghan and Park, Jongchan and Lee, Hayeon and Park, Suhwan and Nam, Yunjea and Kwon, Gihyeon and Jung, Woohyeon and Lee, Youngmoon},
  booktitle = {ICROS 2023},
  year = {2023},
  url = {https://github.com/7drone/carrier_ros},
}
```

## License

MIT
