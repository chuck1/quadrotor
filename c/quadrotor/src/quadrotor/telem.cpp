
#include <stdio.h>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat33.h>
#include <math/mat44.h>

#include <quadrotor/telem.h>
#include <quadrotor/plant.h>
#include <quadrotor/quadrotor.h>


Telem::Telem(Quadrotor* quad):
	quad_(quad)
{
	int n = quad_->N_;

	// state variables
	q_.alloc(n);
	o_.alloc(n);
	//a_.alloc(n);

	x_.alloc(n);
	v_.alloc(n);
	//al_.alloc(n);
}

void Telem::step(int ti) {
	double dt = quad_->t_[ti] - quad_->t_[ti-1];

	// rotation
	o_[ti] = o_[ti-1] + quad_->plant_->od_[ti] * dt;
	
	if(o_[ti].isNan()) {
		
		printf("o  "); o_[ti].print();
		throw;
	}
	
	double o_magn = o_[ti].magnitude();
	
	if(o_magn > 1e6) {
		//throw OmegaHigh(ti);
	}

	math::quat r;
	
	if (o_magn == 0.0) {
		r = math::quat();
	} else {
		math::vec3 o_hat = o_[ti] / o_magn;
		r = math::quat(o_magn * dt, o_hat);
	}
	
	if(!r.isSane()) {
		printf("o_ "); o_[ti].print();
		printf("o_magn %f\n", o_magn);
		throw;
	}
	
	q_[ti] = r * q_[ti-1];
	
	// translation
	v_[ti] = v_[ti-1] + quad_->plant_->a_[ti] * dt;
	x_[ti] = x_[ti-1] + v_[ti] * dt;
	
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
	q_.write(file, n);
	o_.write(file, n);

	fclose(file);
}


