#ifndef __CONTROL_LAW_JERK__
#define __CONTROL_LAW_JERK__

#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

class Jerk: public ControlLaw {
	public:
		Jerk(Quadrotor*);
		
		virtual void	step(int, double) = 0;
		
	public:
		Array<math::vec3>	jerk_;
};
class JerkPos: public Jerk {
	public:
		JerkPos(Quadrotor*);

		void		step(int, double);
	public:
		math::mat33	C_[4];
		
		Array<math::vec3>	e_[4];
		Array<math::vec3>	x_ref_[4];
};

Jerk::Jerk(Quadrotor* r): ControlLaw(r) {}

void	JerkPos::step(int i, double h) {
	
	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);
	
	
	e_[1][i] = r_->x(i) - x_ref_[0][i];
	e_[2][i] = r_->v(i) - x_ref_[1][i];
	e_[3][i] = r_->a(i) - x_ref_[2][i];

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;
	

	jerk_[i] = 
		C_[0] * e_[0][i] + 
		C_[1] * e_[1][i] + 
		C_[2] * e_[2][i] + 
		C_[3] * e_[3][i] +
		x_ref_[3][i];

}

#endif

