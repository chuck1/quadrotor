

#include <math/quat.h>
#include <math/vec3.h>

#include <stdio.h>

#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>

Command::Base::Base(Command::Base::Type::e type, Command::Base::Mode::e mode, Quadrotor* r):
	r_(r),
	flag_(0),
	mode_(mode),
	type_(type)
{
}

Command::Position::Position(Command::Base::Type::e type, Command::Base::Mode::e mode, Quadrotor* r):
	Command::Base(type, mode, r)
{
}

Command::Position::Position(Command::Base::Type::e type, Command::Base::Mode::e mode, math::vec3 thresh, Quadrotor* r):
	Command::Base(type, mode, r),
	thresh_(thresh)
{
}

Command::Point::Point(math::vec3 x2, math::vec3 thresh, Quadrotor* r):
	Command::Position(Command::Base::Type::POINT, Command::Position::Mode::NORMAL, thresh, r),
	x2_(x2)
{
}

Command::Point::Point(math::vec3 x2, Quadrotor* r):
	Command::Position(Command::Base::Type::POINT, Command::Position::Mode::HOLD, r),
	x2_(x2)
{
}
void Command::Point::check(int ti) {

	math::vec3 tol(0.01,0.01,0.01);
	
	::Position* pos = r_->brain_->pos_;

	if(mode_ == Command::Position::Mode::NORMAL) {

		bool close = pos->e1_[ti].abs_less(thresh_);

		if (close) {

			if (pos->e2_[ti].abs_less(tol)) {

				if (pos->e3_[ti].abs_less(tol)) {

					if (pos->e4_[ti].abs_less(tol)) {

						if(pos->jounce_[ti-1].abs_less(tol)) {
							settle(ti, r_->t_[ti]);
						}
					}
				}
			}
		}
	}
}
void Command::Point::settle(int ti, double t) {

	if(!(flag_ & Command::Position::Flag::COMPLETE)) {
		ts_ = t;
		ti_s_ = ti;

		flag_ |= Command::Position::Flag::COMPLETE;

		printf("settled %f\n", t);
	}

}

Command::Path::Path(math::vec3 (*f)(double), Quadrotor* r):
	Position(Command::Base::Type::PATH, Command::Base::Mode::HOLD, r),
	f_(f)
{
	printf("path %p\n",this);
	printf("f %p\n",f_);

	if (f_ == NULL) throw;
}

Command::Orient::Orient(math::quat q, double thresh, Quadrotor* r):
	Base(Base::Type::ORIENT, Base::Mode::NORMAL, r),
	q_(q),
	thresh_(thresh)
{
}

Command::Orient::Orient(math::quat q, Quadrotor* r):
	Base(Base::Type::ORIENT, Base::Mode::HOLD, r)
{
}









