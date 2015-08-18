
#include <stdio.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <math/math.hpp>

#include <drone/util/print.hpp>
#include <drone/Telem.hpp>
#include <drone/Plant.hpp>
#include <drone/Drone.hpp>


Telem::Telem(Quadrotor* quad):
	quad_(quad)
{
	if(quad) {
	
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
}
void			Telem::step(int ti, float dt)
{
	//float dt = quad_->t_[ti] - quad_->t_[ti-1];


	// rotation
	omega_[ti] = omega_[ti-1] + alpha_[ti-1] * dt;
	
	if(quad_->isset_debug()) {
	printf("telemetry\n");
	printf("  alpha(i-1)  %16e%16e%16e\n", alpha_[ti-1].x, alpha_[ti-1].y, alpha_[ti-1].z);
	printf("  omega(i-1)  %16e%16e%16e\n", omega_[ti-1].x, omega_[ti-1].y, omega_[ti-1].z);
	printf("  omega(i)    %16e%16e%16e\n", omega_[ti].x, omega_[ti].y, omega_[ti].z);
	}

	if(math::is_nan_or_inf(omega_[ti])) {
		//printf("o  "); omega_[ti].print();
		throw 0;
	}
	
	float o_magn = glm::length(omega_[ti]);
	
	if(o_magn > 1e6) {
		//throw OmegaHigh(ti);
		throw 0;
	}

	// construct a quaternion rotation based on rotational velocity
	glm::quat r;
	
	if (o_magn < 1e-6) {
		r = glm::quat();
	} else {
		float angle = o_magn * dt;

		r = glm::quat(cos(angle/2.f), (float)sin(angle/2.f) * glm::normalize(omega_[ti]));
	}
	
	if(math::is_nan_or_inf(r)) {
		//printf("o_ "); omega_[ti].print();
		//printf("o_magn %f\n", o_magn);
		throw 0;
	}
	
	//q_[ti] = q_[ti-1] * r;
	q_[ti] = glm::normalize(r * q_[ti-1]);
	//q_[ti] = glm::cross(r, q_[ti-1]);

#if 0
	if(0){
	printf("apply omega\n");
	print(q_[ti-1]);
	print(q_[ti]);
	print(r);
	}
	if(0) {
	printf("quat mul 0\n");
	print(r * q_[ti-1]);
	print(glm::length(r * q_[ti-1]));
	print(glm::cross(r, q_[ti-1]));
	print(glm::dot(r, q_[ti-1]));
	printf("quat mul 1\n");
	}
#endif
	// translation
	
	v_[ti] = v_[ti-1] + a_[ti-1] * dt;
	x_[ti] = x_[ti-1] + v_[ti-1] * dt;
	
	jerk_[ti-1] = (a_[ti-1] - a_[ti-2]) / dt;
	s_[ti-1] = (jerk_[ti-1] - jerk_[ti-2]) / dt;
}
void			Telem::read()
{
	FILE* file = fopen("data/telem.txt","r");

	x_.read(file);
	v_.read(file);
	a_.read(file);
	jerk_.read(file);
	s_.read(file);
	q_.read(file);
	omega_.read(file);

	fclose(file);
}
void			Telem::write(int n)
{
	FILE* file = fopen("data/telem.txt","w");

	n = (n > 0) ? (n) : (quad_->N_);
/*
:glm::vec3* e1 = new glm::vec3[n];
	glm::vec3* q = new glm::vec3[n];
	glm::vec3* q_ref = new glm::vec3[n];

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


