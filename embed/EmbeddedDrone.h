#ifndef E_DRONE_H
#define E_DRONE_H

#include <drone/Drone.hpp>

class AccelGyro;

class EmbeddedDrone
{
public:
	void	communicate();
	void	measure();
	void	set_motor_speed();
	
	std::shared_ptr<Drone>	_M_drone;
	
	AccelGyro *		_M_ag;
};

#endif
