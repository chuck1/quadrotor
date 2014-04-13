

#include <math/quat.h>
#include <math/vec3.h>

#include <stdio.h>

#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

Command::Base::Base(Quadrotor* r, Command::Base::Type::e type):
	r_(r),
	flag_(0),
	type_(type)
{
}
void	Command::Base::Check(int i) {
	for(auto it = stop_.begin(); it != stop_.end(); ++it) {
		(*it)->Check(i);
	}
}
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
Command::X::X(Quadrotor* r, Input::Vec3* in):
	Base(r, Command::Base::Type::X),
	in_(in)
{
	printf("in %p\n",in_);

	if (in_ == NULL) throw;
}
Command::V::V(Quadrotor* r, Input::Vec3* in):
	Base(r, Command::Base::Type::V),
	in_(in)
{
	printf("in %p\n",in_);

	if (in_ == NULL) throw;
}
Command::Q::Q(Quadrotor* r, Input::Quat* in):
	Base(r, Base::Type::Q),
	in_(in)
{
}










