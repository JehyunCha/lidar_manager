#include "lidar_manager/lidar_parser.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <iostream>


LidarParser::LidarParser(const rclcpp::NodeOptions& node_options)
    : Node("lidar_parser", node_options)
{
    RCLCPP_INFO(this->get_logger(), "LidarParser node has been started.");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V(rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile());

    m_lidarSubscriber = this->create_subscription<PointCloud2>(
        "pointcloud",
        QOS_RKL10V,
        [this](const PointCloud2::SharedPtr msg) -> void
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
            while (iter_x != iter_x.end()) {
                // TODO: do something with the values of x, y, z
                std::cout << *iter_x << ", " << *iter_y << ", " << *iter_z << std::endl;
                ++iter_x; ++iter_y; ++iter_z;
            }
        }
    );
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto lidarParserNode = std::make_shared<LidarParser>();

  rclcpp::spin(lidarParserNode);

  rclcpp::shutdown();

  return 0;
}