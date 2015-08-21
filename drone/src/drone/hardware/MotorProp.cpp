#include <cmath>

#include <drone/hardware/MotorProp.hpp>

float	drone::hardware::MotorProp::thrust_max()
{
	float f = 200;

	float p = 0.005;
	
	float r = 0.05;

	float fp = f * p;

	float V0 = 0;

	float th = 1.225 * M_PI * r * r * fp * (fp - V0) * pow((r / 1.64773 / p), 1.5);

	return th;
}

