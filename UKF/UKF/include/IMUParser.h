#pragma once
#include <cmath>

class IMUParser
{
private:
	double yaw;
	double yaw_dot;
	double lastTimeStamp;
	void keepYawInAbsolute180();

public:

	template<typename StringType>
	void parseMessage(const StringType & message);
	
	template<typename StringType>
	void parseData(const StringType & dataSegment);

	double getYaw();
	double getYaw_dot();
};

template<typename StringType>
void IMUParser::parseMessage(const StringType & message)
{


	keepYawInAbsolute180();
}

template<typename StringType>
void IMUParser::parseData(const StringType & dataSegment)
{

}

double IMUParser::getYaw()
{
	
	return yaw;
}

double IMUParser::getYaw_dot()
{
	return yaw_dot;
}

void IMUParser::keepYawInAbsolute180()
{
	if (yaw > M_PI)
	{
		yaw -= M_PI;
	}
	if(yaw < -M_PI)
	{
		yaw += M_PI;
	}
}