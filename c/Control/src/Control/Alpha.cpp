
#include <quadrotor/command.h>
#include <quadrotor/fda.h>
#include <quadrotor/Input.hpp>
#include <quadrotor/ControlLaw/Alpha.h>

Alpha1::Base::Base(Quadrotor* r): CL::Base(r), CL::Alpha(r) {
	alloc(r_->N_);
}
void	Alpha1::Base::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	CL::Alpha::alloc(n);
}
void	Alpha1::Base::write(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	CL::Alpha::write(n);
}
void	Alpha1::Base::Step(int i, double h) {
	CL::Alpha::Step(i,h);
}

void	Alpha1::Q::Step(int i, double h) {
	
	Command::Q* q = dynamic_cast<Command::Q*>(command_);

	if(i == 0) {	
		// back fill
		q_ref_[-1] = q->in_->f(r_->t(-1));
		q_ref_[-2] = q->in_->f(r_->t(-2));
		q_ref_[-3] = q->in_->f(r_->t(-3));
	}

	q_ref_[i] = q->in_->f(r_->t(i));

	// reference derivative
	
	forward_quavec(q_ref_, q_ref__[0], h, i);

	forward(q_ref__[0], q_ref__[1], h, i);

	// error

	e_[1][i] = (q_ref_[i] * r_->q(i).getConjugate()).getImaginaryPart();

	e_[2][i] = q_ref__[0][i] - r_->omega(i);
	
	e_[0][i] = e_[0][i-1] + e_[1][i] * h;


	// control

	alpha_[i] = 
		c_[0] * e_[0][i] + 
		c_[1] * e_[1][i] + 
		c_[2] * e_[2][i] + 
		q_ref__[1][i];

	Alpha1::Base::Step(i,h);
	
/*
	// reference derivatives
	forward_quavec(q_ref_, q_ref_d_,  dt, ti, ti_0, 0);
	
	forward(q_ref_d_, q_ref_dd_, dt, ti);
	
	if(!q_ref_dd_[ti].isSane()) {
		printf("insane\n");
		q_ref_d_[ti].print();
		q_ref_dd_[ti].print();
		throw;
	}
	
	// errors and their magnitudes	

	switch(mode_) {
		case Attitude::Mode::e::VEL:
			e2_[ti] = o_ref_[ti] - quad_->telem_->o_[ti];

			if(!e2_[ti].isSane()) {
				printf("e2\n");
				e2_[ti].print();
				printf("o_ref\n");
				o_ref_[ti].print();
				printf("o\n");
				quad_->telem_->o_[ti].print();
				throw;
			}

			break;
		case Attitude::Mode::e::ATT:
			

			e1_mag_[ti] = e1_[ti].getImaginaryPart().magnitude();

			e2_[ti] = q_ref_d_[ti] - quad_->telem_->o_[ti];
			break;
	}

	// magnitude derivatives
	forward(e1_mag_,  e1_mag_d_, dt, ti);

	// check
	if (ti_0 > 0) {
		if (att_) {
			if (att_->mode_ == Command::Base::Mode::NORMAL) {
				if ((e1_mag_d_[ti] < 0.0) && (e1_mag_d_[ti] > -0.001)) {
					if (e1_[ti].getAngle() < att_->thresh_) {
						att_->flag_ |= Command::Base::Flag::COMPLETE;
					}
				}
			}
		}
	}
	*/
}
void	Alpha1::Q::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	Alpha1::Base::alloc(n);
	CL::Q<2>::alloc(n);
}
void	Alpha1::Q::write(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	Alpha1::Base::write(n);
	CL::Q<2>::write(n);
}
bool	Alpha1::Q::Check(int i, math::vec3 tol) {

	if(e_[0][i].Abs() < tol) {
		if(e_[1][i].Abs() < tol) {
			if(e_[2][i].Abs() < tol) {
				return true;
			}
		}
	}
	return false;
}


void	Alpha1::Omega::Step(int i, double h) {

	alpha_[i] = 
		c_[0] * e_[0][i] +
		c_[1] * e_[1][i] +
		omega_ref_[1][i];

}
bool	Alpha1::Omega::Check(int, math::vec3) {
	return false;
}
void	Alpha1::Omega::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	Alpha1::Base::alloc(n);
	CL::Omega<2>::alloc(n);
}
void	Alpha1::Omega::write(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	Alpha1::Base::write(n);
	CL::Omega<2>::write(n);
}



