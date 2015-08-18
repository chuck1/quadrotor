

#include <glm/glm.hpp>

#include <stdio.h>

#include <drone/command/command.h>
#include <drone/command/Stop.hpp>
#include <drone/command/Input.hpp>
#include <drone/Drone.hpp>
#include <drone/brain.h>
//#include <drone/position.h>
#include <drone/cl/ControlLaw.h>

Command::Q::Q(Quadrotor* r, Input::Quat* in):
	Base(r, Base::Type::Q),
	in_(in)
{
}



