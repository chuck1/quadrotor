
#include <quadrotor/quadrotor.h>
#include <quadrotor/command.h>
#include <quadrotor/except.h>
#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Jounce.h>

Jounce::Base::Base(Quadrotor* r): CL::Base(r), CL::Thrust(r), CL::Alpha(r) {}

Jounce::X::X(Quadrotor* r): CL::Base(r), CL::X(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {}

void	Jounce::X::Check(int i) {

	math::vec3 tol(0.01,0.01,0.01);
	
	Command::X* command = (Command::X*)command_;

	if(command_->mode_ == Command::Base::Mode::NORMAL) {

		if(e_[1][i].Abs() < command->thresh_) {

			if(e_[2][i].Abs() < tol) {

				if(e_[3][i].Abs() < tol) {

					if(e_[4][i].Abs() < tol) {

						if(jounce_[i-1].Abs() < tol) {
							command->Settle(i, r_->t_[i]);
						}
					}
				}
			}
		}
	}
}
void	Jounce::Base::Step(int i, double h) {
	math::vec3 tmp = r_->q(i).rotate(jounce_[i] * r_->m_);

	math::vec3& o = r_->omega(i);

	// thrust
	thrust_[i] = (tmp.z() - (thrust_[i-2] - 2.0 * thrust_[i-1]) / h / h) / (1.0 / h / h - o.x() * o.x() - o.y() * o.y());

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
		alpha_[i].y() = (thrust_[i] * o.x() * o.z() - 2.0 * thrust_d * o.y() - tmp.x()) / thrust_[i];
		alpha_[i].x() = -(thrust_[i] * o.y() * o.z() + 2.0 * thrust_d * o.x() - tmp.y()) / thrust_[i];
	}
	
	CL::Thrust::Step(i,h);
	CL::Alpha::Step(i,h);

	/*
	if(!o.isSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}*/

}
void	Jounce::X::Step(int i, double h) {

	printf("%s\n",__PRETTY_FUNCTION__);

	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);
	forward(x_ref_[3], x_ref_[4], h, i);

	e_[1][i] = r_->x(i) - x_ref_[0][i];
	e_[2][i] = r_->v(i) - x_ref_[1][i];
	e_[3][i] = r_->a(i) - x_ref_[2][i];
	e_[4][i] = r_->jerk(i) - x_ref_[3][i];

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	jounce_[i] = 
		C_[0] * e_[0][i] + 
		C_[1] * e_[1][i] + 
		C_[2] * e_[2][i] + 
		C_[3] * e_[3][i] +
		C_[4] * e_[4][i] +
		x_ref_[4][i];

	Jounce::Base::Step(i, h);
}
void	Jounce::V::Step(int i, double h) {

	forward(v_ref_[0], v_ref_[1], h, i);
	forward(v_ref_[1], v_ref_[2], h, i);
	forward(v_ref_[2], v_ref_[3], h, i);

	e_[1][i] = r_->v(i) - v_ref_[1][i];
	e_[2][i] = r_->a(i) - v_ref_[2][i];
	e_[3][i] = r_->jerk(i) - v_ref_[3][i];

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	jounce_[i] = 
		C_[0] * e_[0][i] + 
		C_[1] * e_[1][i] + 
		C_[2] * e_[2][i] + 
		C_[3] * e_[3][i] +
		v_ref_[3][i];
	
	Jounce::Base::Step(i, h);
}
