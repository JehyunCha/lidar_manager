#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarParser : public rclcpp::Node
{
public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;

private:
    rclcpp::Subscription<PointCloud2>::SharedPtr m_lidarSubscriber;

public:
    LidarParser(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    virtual ~LidarParser() = default;
};