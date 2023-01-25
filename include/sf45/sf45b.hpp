#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ConstForSF45.hpp"
#include "srv/SF45Command.hpp"

#include <vector>
#include <string>

class SF45 : public rclcpp::Node
{
private:
    bool m_isOperated = false;
    int m_numPointsPerMsg = 0;
    int m_currentPoint = 0;

    std::vector<lwDistanceResult> m_distanceResults(maxPointsPerMsg);
	std::vector<rawDistanceResult> m_rawDistances(maxPointsPerMsg);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidarPublisher;
    rclcpp::Service<srv::SF45Command>::SharedPtr m_commandServer;

    sensor_msgs::msg::PointCloud2 m_pointCloudMsg;

    lwSerialPort* m_serial = nullptr;

public:
    explicit SF45(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    virtual ~SF45() = default;

    void initialize_point_cloud_message(const std::string& frameId, const int32_t& maxPointsPerMsg);
    bool initialize_serial_driver(const char* PortName, int32_t BaudRate);
    bool initialize_lidar(lwSf45Params* Params);
    bool parse_received_measurement(lwDistanceResult *DistanceResult, rawDistanceResult *RawDistanceResult);

    void publish_lidar_data();

    bool execute_command(const uint16_t& command);
};