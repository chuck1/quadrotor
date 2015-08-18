

#include <glm/glm.hpp>

#include <stdio.h>

#include <drone/command/command.h>
#include <drone/command/Stop.hpp>
#include <drone/command/Input.hpp>
#include <drone/Drone.hpp>
#include <drone/brain.h>
//#include <drone/position.h>
#include <drone/cl/ControlLaw.h>

Command::V::V(Quadrotor* r, Input::Vec3::Base* in):
	Base(r, Command::Base::Type::V),
	in_(in)
{
	if (in_ == NULL) throw;
}






