
#include <stdio.h>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat33.h>
#include <math/mat44.h>

#include "quadrotor/telem.h"
#include "quadrotor/plant.h"
#include "quadrotor/quadrotor.h"


Telem::Telem(Quadrotor* quad):
	quad_(quad)
{
	int n = quad_->N_;

	// state variables

	q_.alloc(n);
	omega_.alloc(n);
	alpha_.alloc(n);
	
	x_.alloc(n);
	v_.alloc(n);
	a_.alloc(n);
	jerk_.alloc(n);
	s_.alloc(n);

}

void Telem::step(int ti, double dt) {
	//double dt = quad_->t_[ti] - quad_->t_[ti-1];

	// rotation
	omega_[ti] = omega_[ti-1] + alpha_[ti-1] * dt;
	
	
	if(omega_[ti].IsNan()) {
		
		printf("o  "); omega_[ti].print();
		throw;
	}
	
	double o_magn = omega_[ti].magnitude();
	
	if(o_magn > 1e6) {
		//throw OmegaHigh(ti);
	}

	math::quat r;
	
	if (o_magn < 1e-6) {
		r = math::quat();
	} else {
		r = math::quat(o_magn * dt, omega_[ti]);
	}
	
	if(!r.isSane()) {
		printf("o_ "); omega_[ti].print();
		printf("o_magn %f\n", o_magn);
		throw;
	}
	
	q_[ti] = r * q_[ti-1];
	
	// translation
	
	v_[ti] = v_[ti-1] + a_[ti-1] * dt;
	x_[ti] = x_[ti-1] + v_[ti-1] * dt;
	
	jerk_[ti-1] = (a_[ti-1] - a_[ti-2]) / dt;
	s_[ti-1] = (jerk_[ti-1] - jerk_[ti-2]) / dt;
}
void Telem::write(int n) {
	FILE* file = fopen("data/telem.txt","w");

	n = (n > 0) ? (n) : (quad_->N_);
/*
:math::vec3* e1 = new math::vec3[n];
	math::vec3* q = new math::vec3[n];
	math::vec3* q_ref = new math::vec3[n];

	for(int ti = 0; ti < n; ti++) {
		e1[ti] = e1_[ti].getImaginaryPart();
		q[ti] = quad_->telem_->q_[ti].getImaginaryPart();
		q_ref[ti] = q_ref_[ti].getImaginaryPart();
	}
*/	
	
	x_.write(file, n);
	v_.write(file, n);
	a_.write(file, n);
	jerk_.write(file, n);
	s_.write(file, n);

	q_.write(file, n);
	omega_.write(file, n);


	fclose(file);
}


