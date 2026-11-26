#include "input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <fstream>
#include <iostream>
// Raw Ethernet live observer or replay. No ROS1 subscriptions/hardware commands.
int main(int argc,char** argv){try{
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("lt_r2_native_observer");
 auto input=node->declare_parameter<std::string>("capture","");auto interface=node->declare_parameter<std::string>("passive_interface","");
 double seconds=node->declare_parameter<double>("duration",15.0);
 auto pub=node->create_publisher<sensor_msgs::msg::LaserScan>("/candidate/scan",rclcpp::SensorDataQoS());
 lidar::Input source(input,interface,seconds);lidar::Assembler a;lidar::Scan scan;unsigned packets=0,scans=0;
 std::vector<uint8_t> p;double t;
 while(rclcpp::ok()&&source.next(p,t)){const uint8_t* data;size_t size;if(!lidar::udp(p,data,size))continue;
 auto b=lidar::decode(data,size);packets++;if(!a.add(b,t,scan))continue;
 sensor_msgs::msg::LaserScan m;m.header.frame_id="scan";m.header.stamp=rclcpp::Time(int64_t(scan.stamp*1e9));m.angle_min=scan.angle_min;m.angle_max=scan.angle_max;m.angle_increment=(scan.angle_max-scan.angle_min)/(scan.ranges.size()-1);m.time_increment=scan.increment;m.scan_time=scan.period;m.range_min=.05;m.range_max=40;m.ranges=scan.ranges;m.intensities=scan.intensities;pub->publish(m);rclcpp::spin_some(node);scans++;
 }
 std::cout<<"{\"raw_packets\":"<<packets<<",\"published_scans\":"<<scans<<",\"ros1_subscriptions\":0,\"hardware_commands\":0}\n";rclcpp::shutdown();return scans?0:2;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
