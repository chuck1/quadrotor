

#include <math/quat.h>
#include <math/vec3.h>

#include <stdio.h>

#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

Command::Base::Base(Command::Base::Type::e type, Command::Base::Mode::e mode, Quadrotor* r):
	r_(r),
	flag_(0),
	mode_(mode),
	type_(type)
{
}

void Command::X::Settle(int i, double t) {

	if(!(flag_ & Command::Base::Flag::COMPLETE)) {
		ts_ = t;
		ti_s_ = i;

		flag_ |= Command::Base::Flag::COMPLETE;

		printf("settled %f\n", t);
	}

}

Command::X::X(Quadrotor* r, math::vec3 (*f)(double)):
	Base(Command::Base::Type::X, Command::Base::Mode::HOLD, r),
	f_(f)
{
	printf("f %p\n",f_);

	if (f_ == NULL) throw;
}
Command::X::X(Quadrotor* r, math::vec3 (*f)(double), math::vec3 const & thresh):
	Base(Command::Base::Type::X, Command::Base::Mode::NORMAL, r),
	f_(f),
	thresh_(thresh)
{
	printf("f %p\n",f_);

	if (f_ == NULL) throw;
}

Command::Q::Q(Quadrotor* r, math::quat (*f)(double), double thresh):
	Base(Base::Type::Q, Base::Mode::NORMAL, r),
	f_(f),
	thresh_(thresh)
{
}

Command::Q::Q(Quadrotor* r, math::quat (*f)(double)):
	Base(Base::Type::Q, Base::Mode::HOLD, r)
{
}









