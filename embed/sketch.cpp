
#include <EmbeddedDrone.h>
#include <AccelGyro.h>

EmbeddedDrone edrone;

AccelGyro ag;

void setup()
{

	edrone._M_ag = &ag;

}

void loop()
{

	edrone.communicate();
	edrone.measure();
	edrone.set_motor_speed();

}

