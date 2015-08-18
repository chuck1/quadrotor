

#include <glm/glm.hpp>

#include <stdio.h>

#include <drone/command/command.h>
#include <drone/command/Input.hpp>
#include <drone/command/Stop.hpp>
#include <drone/Drone.hpp>
#include <drone/brain.h>
//#include <drone/position.h>
#include <drone/cl/ControlLaw.h>

/*
void Command::X::Settle(int i, double t) {

	if(!(flag_ & Command::Base::Flag::COMPLETE)) {
		ts_ = t;
		ti_s_ = i;

		flag_ |= Command::Base::Flag::COMPLETE;

		printf("settled %f\n", t);
	}

}
*/
Command::X::X(Quadrotor* r, Input::Vec3::Base* in):
	Base(r, Command::Base::Type::X),
	in_(in)
{
	if (in_ == NULL) throw;
}








