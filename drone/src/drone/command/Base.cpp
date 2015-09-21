

#include <glm/glm.hpp>

#include <stdio.h>

#include <drone/command/command.h>
#include <drone/command/Input.hpp>
#include <drone/command/Stop.hpp>
#include <drone/Drone.hpp>
#include <drone/Brain.hpp>
//#include <drone/position.h>
#include <drone/cl/ControlLaw.h>

Command::Base::Base(Drone* r, Command::Base::Type::e type):
	r_(r),
	flag_(0),
	type_(type)
{
}
void	Command::Base::check(int i)
{
	for(auto it = stop_.begin(); it != stop_.end(); ++it) {
		(*it)->check(i);
	}
}




