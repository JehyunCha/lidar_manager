#pragma once

std::map<int, int> updateRateMap = {
		{1, 50},
		{2, 200},
		{3, 200},
		{4, 400},
		{5, 500},
		{6, 625},
		{7, 1000},
		{8, 1250},
		{9, 1538},
		{10, 2000},
		{11, 2500},
		{12, 5000},
};

struct lwSf45Params {
	int32_t updateRate;
	int32_t cycleDelay;
	float lowAngleLimit;
	float highAngleLimit;
};


struct lwDistanceResult {
	float x;
	float y;
	float z;
};

struct rawDistanceResult
{
	float distance;
	float angle;
};



// You may refer here for specific commands: https://support.lightware.co.za/sf45b/#/commands - JHCha at 221206
enum class SF45_command
{
	ProductName = 0,
	Token = 10,
	SaveParameter = 12,
	Reset = 14,
	DistanceOutput = 27,
	Stream = 30,
	DistanceDataInCm = 44,
	DistanceDataInMm = 45,
	UpdateRate = 66,
	BaudRate = 79,
	ScanSpeed = 85,
	ScanEnable = 96,
	ScanPosition = 97,
	None = 100
};