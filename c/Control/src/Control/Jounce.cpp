
#include <quadrotor/quadrotor.h>
#include <quadrotor/command.h>
#include <quadrotor/except.h>
#include <quadrotor/fda.h>
#include <quadrotor/Input.hpp>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Jounce.h>

Jounce::Base::Base(Quadrotor* r): CL::Base(r), CL::Thrust(r), CL::Alpha(r) {}


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
	if(thrust_[i] != 0) {
		alpha_[i].y() = (thrust_[i] * o.x() * o.z() - 2.0 * thrust_d * o.y() - tmp.x()) / thrust_[i];
		alpha_[i].x() = -(thrust_[i] * o.y() * o.z() + 2.0 * thrust_d * o.x() - tmp.y()) / thrust_[i];
	}
	
	alpha_[i].print();

	CL::Thrust::Step(i,h);
	CL::Alpha::Step(i,h);
	
	/*
	if(!o.isSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}*/

}
void	Jounce::Base::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	CL::Thrust::alloc(n);
	CL::Alpha::alloc(n);
	jounce_.alloc(n);
}
void	Jounce::Base::write(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	FILE* file = fopen("data/jounce.txt","w");
	jounce_.write(file,n);
	fclose(file);

	CL::Thrust::write(n);
	CL::Alpha::write(n);
}

Jounce::X::X(Quadrotor* r): CL::Base(r), CL::X<5>(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {
	printf("%s\n",__PRETTY_FUNCTION__);
	alloc(r->N_);

}
bool	Jounce::X::Check(int i, math::vec3 tol) {
	//printf("%s\n",__PRETTY_FUNCTION__);

	//Command::X* command = (Command::X*)command_;

	if(e_[1][i].Abs() < tol) {
		if(e_[2][i].Abs() < tol) {
			if(e_[3][i].Abs() < tol) {
				if(e_[4][i].Abs() < tol) {
					if(jounce_[i-1].Abs() < tol) {
						//command->Settle(i, r_->t_[i]);
						return true;
					}
				}
			}
		}
	}
	return false;
}

void	Jounce::X::Step(int i, double h) {
	//printf("%s\n",__PRETTY_FUNCTION__);
	//printf("%f\n",h);

	Command::X* x = dynamic_cast<Command::X*>(command_);

	if(i == 0) {	
		// back fill
		x_ref_[0][-1] = x->in_->f(r_->t(-1));
		x_ref_[0][-2] = x->in_->f(r_->t(-2));
		x_ref_[0][-3] = x->in_->f(r_->t(-3));
	}

	x_ref_[0][i] = x->in_->f(r_->t(i));

	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);
	forward(x_ref_[3], x_ref_[4], h, i);

	e_[1][i] = x_ref_[0][i] - r_->x(i);
	e_[2][i] = x_ref_[1][i] - r_->v(i);
	e_[3][i] = x_ref_[2][i] - r_->a(i-1);
	e_[4][i] = x_ref_[3][i] - r_->jerk(i-1);

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	jounce_[i] = 
		c_[0] * e_[0][i] + 
		c_[1] * e_[1][i] + 
		c_[2] * e_[2][i] + 
		c_[3] * e_[3][i] +
		c_[4] * e_[4][i] +
		x_ref_[4][i];

	Jounce::Base::Step(i, h);

	if(0) {
		printf("e_[2][i]\n");
		e_[2][i].print();
		x_ref_[1][i].print();
		r_->v(i).print();
		printf("x_ref_[0][%i]\n",i);
		x_ref_[0][i].print();
		printf("x_ref_[0][%i]\n",i-1);
		x_ref_[0][i-1].print();
	}
}
void	Jounce::X::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	Jounce::Base::alloc(n);
	CL::X<5>::alloc(n);
}
void	Jounce::X::write(int n) {
	Jounce::Base::write(n);
	CL::X<5>::write(n);
}

Jounce::V::V(Quadrotor* r): CL::Base(r), CL::V<4>(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {
	printf("%s\n",__PRETTY_FUNCTION__);
	alloc(r->N_);
}
void	Jounce::V::Step(int i, double h) {
	//printf("%s\n",__PRETTY_FUNCTION__);

	Command::V* v = dynamic_cast<Command::V*>(command_);

	if(i == 0) {	
		// back fill
		v_ref_[0][-1] = v->in_->f(r_->t(-1));
		v_ref_[0][-2] = v->in_->f(r_->t(-2));
		v_ref_[0][-3] = v->in_->f(r_->t(-3));
	}

	v_ref_[0][i] = v->in_->f(r_->t(i));
	//v_ref_[0][i].print();

	// reference derivatives

	forward(v_ref_[0], v_ref_[1], h, i);
	forward(v_ref_[1], v_ref_[2], h, i);
	forward(v_ref_[2], v_ref_[3], h, i);

	e_[1][i] = v_ref_[0][i] - r_->v(i);
	e_[2][i] = v_ref_[1][i] - r_->a(i);
	e_[3][i] = v_ref_[2][i] - r_->jerk(i);

	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	//c_[1].print();

	jounce_[i] = 
		c_[0] * e_[0][i] + 
		c_[1] * e_[1][i] + 
		c_[2] * e_[2][i] + 
		c_[3] * e_[3][i] +
		v_ref_[3][i];

	//jounce_[i].print();

	Jounce::Base::Step(i, h);
}
bool	Jounce::V::Check(int i, math::vec3 tol) {
	//printf("%s\n",__PRETTY_FUNCTION__);

	if(e_[1][i].Abs() < tol) {
		if(e_[2][i].Abs() < tol) {
			if(e_[3][i].Abs() < tol) {
				if(jounce_[i-1].Abs() < tol) {
					printf("i=%i\n",i);
					e_[1][i].print();
					e_[2][i].print();
					e_[3][i].print();
					return true;
					//command->Settle(i, r_->t_[i]);
				}
			}
		}
	}
	return false;
}
void	Jounce::V::alloc(int n) {
	Jounce::Base::alloc(n);
	CL::V<4>::alloc(n);
}
void	Jounce::V::write(int n) {
	Jounce::Base::write(n);
	CL::V<4>::write(n);
}

