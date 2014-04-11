#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

namespace Command {
	class Base;
}

class ControlLawTT;

class ControlLaw {
	public:
		ControlLaw(Quadrotor* r);

		virtual void	SetCommand(int, Command::Base*) = 0;

		virtual void	Step(int, double);
	public:
		Quadrotor*	r_;

		ControlLawTT*	cltt_;
};


// outputs thrust and x- and y-components of angular acceleration
class ControlLawTT {
	public:
		ControlLawTT(Quadrotor* r);
		
		virtual void	Step(int, double) = 0;
	public:
		Quadrotor*	r_;

		Array<double>		thrust_;
		Array<math::vec3>	alpha_;

};

ControlLaw::ControlLaw(Quadrotor* r): r_(r) {}

void	ControlLaw::Step(int i, double h) {
	
	cltt_->Step(i, h);
	
	math::vec3 torque = r_->angular_accel_to_torque(i, cltt_->alpha_[i]);

	// motor speed

}

#endif
