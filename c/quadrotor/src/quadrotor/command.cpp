

#include <math/quat.h>
#include <math/vec3.h>

#include <stdio.h>

#include <quadrotor/command.h>

Command::Base::Base(Command::Base::Type::e type, Command::Base::Mode::e mode):
	flag_(0),
	mode_(mode),
	type_(type)
{
}

Command::Position::Position(Command::Base::Type::e type, Command::Base::Mode::e mode):
	Command::Base(type, mode)
{
}

Command::Position::Position(Command::Base::Type::e type, Command::Base::Mode::e mode, math::vec3 thresh):
	Command::Base(type, mode),
	thresh_(thresh)
{
}

Command::Move::Move(math::vec3 x2, math::vec3 thresh):
	Command::Position(Command::Base::Type::MOVE, Command::Position::Mode::NORMAL, thresh),
	x2_(x2)
{
}

Command::Move::Move(math::vec3 x2):
	Command::Position(Command::Base::Type::MOVE, Command::Position::Mode::HOLD),
	x2_(x2)
{
}

void Command::Move::settle(int ti, double t) {

	if(!(flag_ & Command::Position::Flag::COMPLETE)) {
		ts_ = t;
		ti_s_ = ti;

		flag_ |= Command::Position::Flag::COMPLETE;

		printf("settled %f\n", t);
	}

}

Command::Path::Path(math::vec3 (*f)(double)):
	Position(Command::Base::Type::PATH, Command::Base::Mode::HOLD),
	f_(f)
{
	printf("path %p\n",this);
	printf("f %p\n",f_);

	if (f_ == NULL) throw;
}

Command::Orient::Orient(math::quat q, double thresh):
	Base(Base::Type::ORIENT, Base::Mode::NORMAL),
	q_(q),
	thresh_(thresh)
{
}

Command::Orient::Orient(math::quat q):
	Base(Base::Type::ORIENT, Base::Mode::HOLD)
{
}









