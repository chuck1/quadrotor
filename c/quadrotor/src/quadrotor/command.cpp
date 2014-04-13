

#include <math/quat.h>
#include <math/vec3.h>

#include <stdio.h>

#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

Command::Base::Base(Quadrotor* r, Command::Base::Type::e type, Command::Base::Mode::e mode, std::vector<Command::Stop::Base*> stop):
	r_(r),
	flag_(0),
	mode_(mode),
	type_(type),
	stop_(stop)
{
}
void	Command::Base::Check(int i) {
	for(auto it = stop_.begin(); it != stop_.end(); ++it) {
		(*it)->Check(i);
	}
}

void Command::X::Settle(int i, double t) {

	if(!(flag_ & Command::Base::Flag::COMPLETE)) {
		ts_ = t;
		ti_s_ = i;

		flag_ |= Command::Base::Flag::COMPLETE;

		printf("settled %f\n", t);
	}

}

Command::X::X(Quadrotor* r, math::vec3 (*f)(double), std::vector<Command::Stop::Base*> stop):
	Base(r, Command::Base::Type::X, Command::Base::Mode::HOLD, stop),
	f_(f)
{
	printf("f %p\n",f_);

	if (f_ == NULL) throw;
}

Command::Q::Q(Quadrotor* r, math::quat (*f)(double), std::vector<Command::Stop::Base*> stop):
	Base(r, Base::Type::Q, Base::Mode::NORMAL, stop),
	f_(f)
{
}










