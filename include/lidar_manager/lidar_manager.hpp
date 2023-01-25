#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "srv/sf45_command.hpp"

class LidarManager : public rclcpp::Node
{
public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using SF45Command = srv::sf45_command;

private:
    rclcpp::Subscription<PointCloud2>::SharedPtr m_lidarSubscriber;

public:
    LidarManager(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    virtual ~LidarManager() = default;
};