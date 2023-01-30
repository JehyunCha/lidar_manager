#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lidar_interface/srv/sf45_command.hpp"

class LidarCommander : public rclcpp::Node
{
public:
    using SF45Command = lidar_interface::srv::SF45Command;

private:
    rclcpp::Client<SF45Command>::SharedPtr m_commandClient;

public:
    LidarCommander(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    virtual ~LidarCommander() = default;

    void send_request();
};