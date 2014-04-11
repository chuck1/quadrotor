
#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Alpha.h>
#include <quadrotor/ControlLaw/Jerk.h>

Jerk::Base::Base(Quadrotor* r): CL::Base(r), CL::Thrust(r), CL::Omega(r), CL::Alpha(r), Alpha1::Base(r), Alpha1::Omega(r)/*, Alpha::Base(r)*/ {}
//: Alpha::Omega(r), CL::Thrust(r) {}
//, CL::Alpha(r) {}

void	Jerk::X::step(int i, double h) {

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
	
	math::vec3 tmp = r_->q(i).rotate(jerk_[i] * r_->m_);
	
	thrust_[i] = thrust_[i-1] + tmp.z() * h;

	math::vec3 o;
	if(thrust_[i] > 0) {
		o.x() = tmp.y() / thrust_[i];
		o.y() = -tmp.x() / thrust_[i];
	}

/*
	if(!o.IsSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}
*/
	
	omega_ref_[0][i] = o;

}


