#include <Odometry.h>
#include <fstream>

std::ofstream OD("Odometry.csv");

void Odometry::sendOdometryAndSteerInfo(long int odometry[4], long int Steer[4], int m)
{
	if (!isInitialized)
	{
		for (int i = 0; i < 4; i++)
		{
			lastOdometry[i] = odometry[i];
			nowOdometry[i] = odometry[i];
		}
		isInitialized = true;
		mode = m;
	}
	else
	{
		mode = m;
		dataFiltering(odometry, Steer);
		calculateDeltaInRelateCoord();
		CalculateWorldState();
		OD << delta.x << " " << delta.y << " " << delta.heading << std::endl;
	}
}

Position Odometry::getDelta()
{
	return delta;
}

Position Odometry::getPureReckoning()
{
	return state;
}

void Odometry::dataFiltering(long int odometry[4], long int Steer[4])
{
	for (int i = 0; i < 4; i++)
	{
		nowOdometry[i] = (fabs(odometry[i] - nowOdometry[i]) < 100) ? odometry[i] : nowOdometry[i];
		nowSteer[i] = Steer[i];
	}
}

void Odometry::updateLastOdometryAndSteer()
{
	double isNeedToUpdate = false;
	if (mode == 1)
	{
		isNeedToUpdate = true;
	}
	else
	{
		if (!(delta.x == 0 && delta.y == 0 && delta.heading == 0))
		{
			isNeedToUpdate = true;
		}
	}
	if (isNeedToUpdate)
	{
		for (int i = 0; i < 4; i++)
		{
			lastOdometry[i] = nowOdometry[i];
		}
	}
}

void Odometry::calculateDeltaInRelateCoord()
{
	switch (mode)
	{
	case 0:
		normalReckoning();
		OD << "normal ";
		break;
	case 1:
		circleReckoning();
		OD << "circle ";
		break;
	default:
		break;
	}
}

void Odometry::circleReckoning()
{
	delta.x = 0;
	delta.y = 0;
	double counter = 0;
	long int d[4];
	for (int i = 0; i < 4; i++)
	{
		d[i] = nowOdometry[i] - lastOdometry[i];
		if (d[i])
		{
			counter += 1.0;
		}
	}
	long int average = (-d[0] + d[1] + d[2] - d[3]) / counter;
	OD << average << " " << counter << " ";
	delta.heading = average * Scale / RadiusOfCircumcircle;
}

void Odometry::normalReckoning()
{
	if (nowSteer[0] > 0 && nowSteer[1] > 0)
	{
		rightTurning();
		OD << "right ";
	}
	else if (nowSteer[0] < 0 && nowSteer[1] < 0)
	{
		leftTurning();
		OD << "left ";
	}
	else
	{
		goStraight();
		OD << "straight ";
	}
}

void Odometry::rightTurning()
{
	double cumulate_theta = 0;
	double tempt_R[2] = { 0 };
	tempt_R[0] = Length / sin(nowSteer[0] * AngleScale);
	tempt_R[1] = Length / tan(nowSteer[0] * AngleScale);
	cumulate_theta -= ((nowOdometry[0] - lastOdometry[0]) / tempt_R[0]);
	cumulate_theta -= ((nowOdometry[3] - lastOdometry[3]) / tempt_R[1]);
	OD << cumulate_theta << " " << tempt_R[0] << " " << tempt_R[1] << " ";
	delta.heading = cumulate_theta * Scale * 0.5;
	delta.x = -(tempt_R[1] + 0.5*Width) * (cos(delta.heading) - 1) - 0.5 * Length * sin(delta.heading);
	delta.y = -(tempt_R[1] + 0.5*Width) * sin(delta.heading) + 0.5 * Length * (cos(delta.heading) - 1);
}

void Odometry::leftTurning()
{
	double cumulate_theta = 0;
	double tempt_R[2] = { 0 };
	tempt_R[0] = Length / sin(-nowSteer[1] * AngleScale);
	tempt_R[1] = Length / tan(-nowSteer[1] * AngleScale);
	cumulate_theta += ((nowOdometry[1] - lastOdometry[1]) / tempt_R[0]);
	cumulate_theta += ((nowOdometry[2] - lastOdometry[2]) / tempt_R[1]);
	OD << cumulate_theta << " " << tempt_R[0] << " " << tempt_R[1] << " ";
	delta.heading = cumulate_theta * Scale * 0.5;
	delta.x = (tempt_R[1] + 0.5*Width) * (cos(delta.heading) - 1) - 0.5 * Length * sin(delta.heading);
	delta.y = (tempt_R[1] + 0.5*Width) * sin(delta.heading) + 0.5 * Length * (cos(delta.heading) - 1);
}

void Odometry::goStraight()
{
	delta.x = 0;
	delta.heading = 0;
	double counter = 0;
	long int d[4];
	for (int i = 0; i < 4; i++)
	{
		d[i] = nowOdometry[i] - lastOdometry[i];
		if (d[i])
		{
			counter += 1.0;
		}
	}
	long int average = (d[0] + d[1] + d[2] + d[3]) / counter;
	OD << average << " " << counter << " ";
	delta.y = average * Scale;
}

void Odometry::CalculateWorldState()
{
	if (mode == 0)
	{
		state.x -= cos(state.heading) * delta.x - sin(state.heading) * delta.y;
		state.y += cos(state.heading) * delta.y - sin(state.heading) * delta.x;
		state.heading -= delta.heading;
	}
	else
	{
		state.heading -= delta.heading;
	}
}