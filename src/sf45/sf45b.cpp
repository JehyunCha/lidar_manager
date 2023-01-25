//----------------------------------------------------------------------------------------------------------------------------------
// LightWare SF45B ROS driver.
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"
#include "ConstForSF45.hpp"
#include "sf45b.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>



SF45b::SF45b(const rclcpp::NodeOptions& node_options)
    : Node("sf45b", node_options)
{
    RCLCPP_INFO(this->get_logger(), "SF45 node has been started.");

	m_lidarPublisher = this->create_publisher<PointCloud2>("pointcloud", 4);

    m_commandServer 
    = this->create_service<SF45Command>(
        "sf45_command", 
        [this](
        const std::shared_ptr<SF45Command::Request> request, 
        const std::shared_ptr<SF45Command::Response> response) -> void
    {
        uint16_t command = request->command;
		bool result = execute_command(command);
        response->result = result;
		std::cout<<"SF45 Command received: "<<result<<std::endl;
    });
}



float degreesToRadians(float degrees) 
{
	return 2 * M_PI * (degrees / 360);
}



void validateParams(lwSf45Params* Params) 
{
	if (Params->updateRate < 1) Params->updateRate = 1;
	else if (Params->updateRate > 12) Params->updateRate = 12;

	if (Params->cycleDelay < 5) Params->cycleDelay = 5;
	else if (Params->cycleDelay > 2000) Params->cycleDelay = 2000;

	if (Params->lowAngleLimit < -160) Params->lowAngleLimit = -160;
	else if (Params->lowAngleLimit > -10) Params->lowAngleLimit = -10;

	if (Params->highAngleLimit < 10) Params->highAngleLimit = 10;
	else if (Params->highAngleLimit > 160) Params->highAngleLimit = 160;
}



 void SF45b::initialize_point_cloud_message(const std::string& frameId, const int32_t& maxPointsPerMsg)
 {
	m_maxPointsPerMsg = maxPointsPerMsg;

	m_pointCloudMsg.header.frame_id = frameId;
	m_pointCloudMsg.height = 1;
	m_pointCloudMsg.width = maxPointsPerMsg;
	
	m_pointCloudMsg.fields.resize(3);
	m_pointCloudMsg.fields[0].name = "x";
	m_pointCloudMsg.fields[0].offset = 0;	
	m_pointCloudMsg.fields[0].datatype = 7;
	m_pointCloudMsg.fields[0].count = 1;

	m_pointCloudMsg.fields[1].name = "y";
	m_pointCloudMsg.fields[1].offset = 4;
	m_pointCloudMsg.fields[1].datatype = 7;
	m_pointCloudMsg.fields[1].count = 1;
	
	m_pointCloudMsg.fields[2].name = "z";
	m_pointCloudMsg.fields[2].offset = 8;
	m_pointCloudMsg.fields[2].datatype = 7;
	m_pointCloudMsg.fields[2].count = 1;

	m_pointCloudMsg.is_bigendian = false;
	m_pointCloudMsg.point_step = 12;
	m_pointCloudMsg.row_step = 12 * m_maxPointsPerMsg;
	m_pointCloudMsg.is_dense = true;

	m_pointCloudMsg.data = std::vector<uint8_t>(m_maxPointsPerMsg * 12);
 }



bool SF45b::initialize_serial_driver(const char* PortName, int32_t BaudRate) 
{
	platformInit();

	m_serial = platformCreateSerialPort();

	if (!m_serial->connect(PortName, BaudRate)) {
		RCLCPP_ERROR(node->get_logger(), "Could not establish serial connection on %s", PortName);
		return false;
	};

	// Disable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(m_serial, 30, 0)) { return 1; }

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(m_serial, 0, modelName)) { return 1; }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(m_serial, 1, &hardwareVersion)) { return 1; }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;	
	if (!lwnxCmdReadUInt32(m_serial, 2, &firmwareVersion)) { return 1; }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(m_serial, 3, serialNumber)) { return 1; }

	RCLCPP_INFO(this->get_logger(), "Model: %.16s", modelName);
	RCLCPP_INFO(this->get_logger(), "Hardware: %d", hardwareVersion);
	RCLCPP_INFO(this->get_logger(), "Firmware: %.16s (%d)", firmwareVersionStr, firmwareVersion);
	RCLCPP_INFO(this->get_logger(), "Serial: %.16s", serialNumber);

	return true;
}



bool SF45b::initialize_lidar(lwSf45Params* Params) 
{
	// Configre distance output for first return and angle. (Command 27: Distance output)
	if (!lwnxCmdWriteUInt32(m_serial, 27, 0x101)) { return false; }

	// (Command 66: Update rate)
	if (!lwnxCmdWriteUInt8(m_serial, 66, Params->updateRate)) { return false; }

	// (Command 85: Scan speed)
	if (!lwnxCmdWriteUInt16(m_serial, 85, Params->cycleDelay)) { return false; }

	// (Command 98: Scan low angle)
	if (!lwnxCmdWriteFloat(m_serial, 98, Params->lowAngleLimit)) { return false; }

	// (Command 99: Scan high angle)
	if (!lwnxCmdWriteFloat(m_serial, 99, Params->highAngleLimit)) { return false; }

	// Enable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(m_serial, 30, 5)) { return false; }

	return true;
}




bool SF45b::parse_received_measurement()
{
	lwDistanceResult distanceResult;
	rawDistanceResult rawDistanceResult;

	// The incoming point data packet is Command 44: Distance data in cm.
	lwResponsePacket response;

	if (lwnxRecvPacket(m_serial, 44, &response, 1000) == 0) 
		return false;

	int16_t distanceCm = (response.data[5] << 8) | response.data[4];
	int16_t angleHundredths = (response.data[7] << 8) | response.data[6];

	float distance = distanceCm / 100.0f;
	float angle = angleHundredths / 100.0f;
	float faceAngle = (angle - 90) * M_PI / 180.0;

	DistanceResult.x = distance * -cos(faceAngle);
	DistanceResult.y = distance * sin(faceAngle);
	DistanceResult.z = 0;

	RawDistanceResult.distance = distance;
	RawDistanceResult.angle = degreesToRadians(angle);

	distanceResults[m_currentPoint] = distanceResult;
	rawDistances[m_currentPoint] = rawDistanceResult;
	++m_currentPoint;


	if (status == 0) {
				break;
			} else {
				
			}

			if (currentPoint == maxPointsPerMsg) {
				memcpy(&pointCloudMsg.data[0], &distanceResults[0], maxPointsPerMsg * 12);
				auto scanTime = node->now().seconds() - pointCloudMsg.header.stamp.sec;

				pointCloudMsg.header.stamp = node->now();
				pointCloudPub->publish(pointCloudMsg);

				if (publishLaserScan)
				{
					auto laserScanMsg = getLaserScanMessage(currentPoint, rawDistances, &params, scanTime, node->get_logger());
					laserScanMsg.header = pointCloudMsg.header;

					laserScanPub->publish(laserScanMsg);
				}

				currentPoint = 0;
			}
}



bool execute_command(const uint16_t& command)
{
	bool result = false;

	switch(command)
	{
	case SF45_command::ScanEnable:
		m_isOperated = !m_isOperated;
		result = lwnxCmdWriteUInt8(m_serial, SF45_command::ScanEnable, m_isOperated);
		std::cout<<"Scan enable command executed"<<std::endl;
		break;
	default:
		break;
	}
}



int main(int argc, char** argv) 
{
	rclcpp::init(argc, argv);

	auto sf45 = std::make_shared<SF45>();

	RCLCPP_INFO(sf45->get_logger(), "Starting SF45B node");
	
	int32_t baudRate = sf45->declare_parameter<int32_t>("baudrate", 115200);
	std::string portName = sf45->declare_parameter<std::string>("port", "/dev/ttyUSB0");
	
	bool result = sf45->initialize_serial_driver(portName.c_str(), baudRate);
	if (result == false) {
		RCLCPP_ERROR(sf45->get_logger(), "Failed to start driver");
		return 1;
	}


	lwSf45Params params;
	params.updateRate = sf45->declare_parameter<int32_t>("updateRate", 6); // 1 to 12
	params.cycleDelay = sf45->declare_parameter<int32_t>("cycleDelay", 5); // 5 to 2000
	params.lowAngleLimit = sf45->declare_parameter<int32_t>("lowAngleLimit", -45.0f); // -160 to -10
	params.highAngleLimit = sf45->declare_parameter<int32_t>("highAngleLimit", 45.0f); // 10 to 160
	validateParams(&params);

	result = initialize_lidar(&params);
	if (result == false) 
	{
		RCLCPP_ERROR(sf45->get_logger(), "Failed to start scan");
		return 1;
	}


	std::string frameId = sf45->declare_parameter<std::string>("frameId", "laser");
	int32_t maxPointsPerMsg = sf45->declare_parameter<int32_t>("maxPoints", 100); // 1 to ...
	if (maxPointsPerMsg < 1) 
		maxPointsPerMsg = 1;
	
	sf45->initialize_point_cloud_message(frameId, maxPointsPerMsg);


	while (rclcpp::ok()) 
	{

		while (true) 
		{
			lwDistanceResult distanceResult;
			rawDistanceResult rawDistanceResult;

			result = sf45->parse_received_measurement(&distanceResult, &rawDistanceResult);

			if (status == 0) {
				break;
			} else {
				distanceResults[currentPoint] = distanceResult;
				rawDistances[currentPoint] = rawDistanceResult;
				++currentPoint;
			}

			if (currentPoint == maxPointsPerMsg) {
				memcpy(&pointCloudMsg.data[0], &distanceResults[0], maxPointsPerMsg * 12);
				auto scanTime = node->now().seconds() - pointCloudMsg.header.stamp.sec;

				pointCloudMsg.header.stamp = node->now();
				pointCloudPub->publish(pointCloudMsg);

				if (publishLaserScan)
				{
					auto laserScanMsg = getLaserScanMessage(currentPoint, rawDistances, &params, scanTime, node->get_logger());
					laserScanMsg.header = pointCloudMsg.header;

					laserScanPub->publish(laserScanMsg);
				}

				currentPoint = 0;
			}
		}

		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();

	return 0;
}