#pragma once
#include <cmath>

struct Position
{
	double x;
	double y;
	double heading;
};

class Odometry
{
private:

	double Length;
	double Width;
	double Perimeter;
	double RadiusOfCircumcircle;
	double Scale;
	double AngleScale;

	long int lastOdometry[4];
	long int nowOdometry[4];
	long int nowSteer[4];
	int mode;
	bool isInitialized = false;
	Position state;
	Position delta;

public:

	Odometry()
	{
		Length = 0.5;
		Width = 0.48;
		Perimeter = 80.110613f;
		RadiusOfCircumcircle = sqrt(pow(Length, 2) + pow(Width, 2));
		Scale = Perimeter / 90.0 / 100.0;
		AngleScale = M_PI / 180.0 / 100.0;
		state = { 0,0,0 };
		delta = { 0,0,0 };
	}

	void sendOdometryAndSteerInfo(long int odometry[4], long int Steer[4], int m);
	Position getDelta();
	Position getPureReckoning();

private:

	void dataFiltering(long int odometry[4], long int Steer[4]);
	void updateLastOdometryAndSteer();
	void calculateDeltaInRelateCoord();
	void circleReckoning();
	void normalReckoning();
	void rightTurning();
	void leftTurning();
	void goStraight();
	void CalculateWorldState();

};