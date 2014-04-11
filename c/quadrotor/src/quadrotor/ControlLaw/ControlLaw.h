#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

namespace Command {
	class Base;
}

class ControlLawTT;

namespace CL {
	class Base {
		public:
			Base(Quadrotor* r);

			virtual void	SetCommand(int, Command::Base*) = 0;

			virtual void	Step(int, double);
		public:
			Quadrotor*	r_;

			Command::Base*	command_;
	};
	class Point: public Base {

	};
	class Path: public Base {

	};
	class Vel: public Base {

	};
	class Orient: public Base {
		
	};
}


// outputs thrust and x- and y-components of angular acceleration
class ControlLawTT {
	public:
		ControlLawTT(Quadrotor* r);

		virtual void	Step(int, double) = 0;
	public:
		ControlLaw*		cl_;

		Array<double>		thrust_;
		Array<math::vec3>	alpha_;

};

CL::Base::Base(Quadrotor* r): r_(r) {}

void	CL::BaseontrolLaw::Step(int i, double h) {

	cltt_->Step(i, h);

	math::vec3 torque = r_->angular_accel_to_torque(i, cltt_->alpha_[i]);

	// motor speed

}

#endif
