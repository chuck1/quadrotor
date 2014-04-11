#ifndef __CONTROL_LAW_JOUNCE__
#define __CONTROL_LAW_JOUNCE__

#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

class Jounce: public ControlLawTT {
	public:
		Jounce(Quadrotor*);
		
		virtual void	step(int) = 0;
		
	public:
		Array<math::vec3>	jounce_;
};
class JouncePoint: virtual public CL::Point, virtual public Jounce {
	public:
		JouncePos(Quadrotor*);

		virtual void	step(int, double);
	public:
		math::mat33	C_[5];
		
		Array<math::vec3>	e_[5];
		Array<math::vec3>	x_ref_[5];
};

Jounce::Jounce(Quadrotor* r): ControlLawTT(r) {}

void Position::fill_xref(int ti1, math::vec3 x) {
	for (int ti = ti1; ti < quad_->N_; ti++) x_ref_[ti] = x;
}
void Position::fill_xref_parametric(int ti1, math::vec3 (*f)(double)) {
	for(int ti = ti1; ti < quad_->N_; ti++) {
		double t = quad_->t_[ti];
		x_ref_[ti] = f(t);
	}
}

void	JouncePos::step(int i, double h) {

	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);
	forward(x_ref_[3], x_ref_[4], h, i);
	
	e_[1][i] = r_->x(i) - x_ref_[0][i];
	e_[2][i] = r_->v(i) - x_ref_[1][i];
	e_[3][i] = r_->a(i) - x_ref_[2][i];
	e_[4][i] = r_->jounce(i) - x_ref_[3][i];

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	jounce_[i] = 
		C_[0] * e_[0][i] + 
		C_[1] * e_[1][i] + 
		C_[2] * e_[2][i] + 
		C_[3] * e_[3][i] +
		C_[4] * e_[4][i] +
		x_ref_[4][i];
	

	
	math::vec3 tmp = r_->q(i).rotate(jounce_[i] * r_->m_);

	math::quat& o = r_->omega(i-1);

	// thrust
	thrust_[i] = (tmp.z - (thrust_[i-2] - 2.0 * thrust_[i-1]) / h / h) / (1.0 / h / h - o.x * o.x - o.y * o.y);

	double thrust_d = (thrust_[i] - thrust_[i-1]) / h;


	if(isnan(thrust_[i]) || isinf(thrust_[i])) {
		//printf("tmp\n");
		//tmp.print();
		//printf("dt %f\n", dt);
		//printf("thrust %f\n", thrust_[i]);
		throw Inf();
	}

	// angular acceleration
	if(thrust_[i] > 0) {
		alpha_[i].y = (thrust_[i] * o.x * o.z - 2.0 * thrust_d * o.y - tmp.x) / thrust_[i];
		alpha_[i].x = -(thrust_[i] * o.y * o.z + 2.0 * thrust_d * o.x - tmp.y) / thrust_[i];
	}

	if(!o.isSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}
}

#endif

