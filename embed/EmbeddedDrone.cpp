#include <AccelGyro.h>
#include <EmbeddedDrone.h>

void	EmbeddedDrone::communicate()
{
	// standard serial with esp01 module
}

void	EmbeddedDrone::measure()
{
	// communicate with MCU 6-axis module
	
	float a[3];
	float alph[3];

	_M_ag->measure(a, alph);
}

void	EmbeddedDrone::set_motor_speed()
{
}


