//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
//#include "ydlidar_ros_driver/LaserFan.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits

#define SDKROSVerision "1.0.2"

CYdLidar laser;

bool stop_scan(std_srvs::Empty::Request &req,
               std_srvs::Empty::Response &res) {
  ROS_DEBUG("Stop scan");
  return laser.turnOff();
}

bool start_scan(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res) {
  ROS_DEBUG("Start scan");
  return laser.turnOn();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ydlidar_ros_driver");
  ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud",
                          1);
//  ros::Publisher laser_fan_pub =
//    nh.advertise<ydlidar_ros_driver::LaserFan>("laser_fan", 1);

  ros::NodeHandle nh_private("~");
  std::string str_optvalue = "/dev/ydlidar";
  nh_private.param<std::string>("port", str_optvalue, "/dev/ydlidar");
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
                    str_optvalue.size());

  ///ignore array
  nh_private.param<std::string>("ignore_array", str_optvalue, "");
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
                    str_optvalue.size());

  std::string frame_id = "laser_frame";
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  nh_private.param<int>("baudrate", optval, 230400);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  nh_private.param<int>("lidar_type", optval, TYPE_TRIANGLE);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  nh_private.param<int>("device_type", optval, YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
  nh_private.param<int>("sample_rate", optval, 9);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  nh_private.param<int>("abnormal_check_count", optval, 4);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  //intensity bit count
  optval = 10;
  nh_private.param<int>("intensity_bit", optval, 10);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  nh_private.param<bool>("resolution_fixed", b_optvalue, true);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  nh_private.param<bool>("reversion", b_optvalue, true);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  nh_private.param<bool>("inverted", b_optvalue, true);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  nh_private.param<bool>("auto_reconnect", b_optvalue, true);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  nh_private.param<bool>("isSingleChannel", b_optvalue, false);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  nh_private.param<bool>("intensity", b_optvalue, false);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  nh_private.param<bool>("support_motor_dtr", b_optvalue, false);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  nh_private.param<float>("angle_max", f_optvalue, 180.f);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  nh_private.param<float>("angle_min", f_optvalue, -180.f);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 16.f;
  nh_private.param<float>("range_max", f_optvalue, 16.f);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  nh_private.param<float>("range_min", f_optvalue, 0.1f);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f;
  nh_private.param<float>("frequency", f_optvalue, 10.f);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  nh_private.param<bool>("invalid_range_is_inf", invalid_range_is_inf,
                         invalid_range_is_inf);

  bool point_cloud_preservative = false;
  nh_private.param<bool>("point_cloud_preservative", point_cloud_preservative,
                         point_cloud_preservative);

  ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan",
                                         stop_scan);
  ros::ServiceServer start_scan_service = nh.advertiseService("start_scan",
                                          start_scan);

  // initialize SDK and LiDAR
  bool ret = laser.initialize();

  if (ret) {//success
    //Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {
    ROS_ERROR("%s\n", laser.DescribeError());
  }

  ros::Rate r(30);

  while (ret && ros::ok()) {
    LaserScan scan;

    if (laser.doProcessSimple(scan)) {
      sensor_msgs::LaserScan scan_msg;
      sensor_msgs::PointCloud pc_msg;
//      ydlidar_ros_driver::LaserFan fan;
      ros::Time start_scan_time;
      start_scan_time.sec = scan.stamp / 1000000000ul;
      start_scan_time.nsec = scan.stamp % 1000000000ul;
      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      pc_msg.header = scan_msg.header;
//      fan.header = scan_msg.header;
      scan_msg.angle_min = (scan.config.min_angle);
      scan_msg.angle_max = (scan.config.max_angle);
      scan_msg.angle_increment = (scan.config.angle_increment);
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = (scan.config.min_range);
      scan_msg.range_max = (scan.config.max_range);
//      fan.angle_min = (scan.config.min_angle);
//      fan.angle_max = (scan.config.max_angle);
//      fan.scan_time = scan.config.scan_time;
//      fan.time_increment = scan.config.time_increment;
//      fan.range_min = (scan.config.min_range);
//      fan.range_max = (scan.config.max_range);

      int size = (scan.config.max_angle - scan.config.min_angle) /
                 scan.config.angle_increment + 1;
      scan_msg.ranges.resize(size,
                             invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
      scan_msg.intensities.resize(size);
      pc_msg.channels.resize(2);
      int idx_intensity = 0;
      pc_msg.channels[idx_intensity].name = "intensities";
      int idx_timestamp = 1;
      pc_msg.channels[idx_timestamp].name = "stamps";

      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                              scan.config.angle_increment);

        if (index >= 0 && index < size) {
          if (scan.points[i].range >= scan.config.min_range) {
            scan_msg.ranges[index] = scan.points[i].range;
            scan_msg.intensities[index] = scan.points[i].intensity;
          }
        }

        if (point_cloud_preservative ||
            (scan.points[i].range >= scan.config.min_range &&
             scan.points[i].range <= scan.config.max_range)) {
          geometry_msgs::Point32 point;
          point.x = scan.points[i].range * cos(scan.points[i].angle);
          point.y = scan.points[i].range * sin(scan.points[i].angle);
          point.z = 0.0;
          pc_msg.points.push_back(point);
          pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
          pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
        }

//        fan.angles.push_back(scan.points[i].angle);
//        fan.ranges.push_back(scan.points[i].range);
//        fan.intensities.push_back(scan.points[i].intensity);
      }

      scan_pub.publish(scan_msg);
      pc_pub.publish(pc_msg);
//      laser_fan_pub.publish(fan);

    } else {
      ROS_ERROR("Failed to get Lidar Data");
    }

    r.sleep();
    ros::spinOnce();
  }

  laser.turnOff();
  ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.disconnecting();
  return 0;
}


