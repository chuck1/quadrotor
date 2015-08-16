/*
#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat33.h>
#include <math/mat44.h>
*/



#include <stdio.h>

#include <algorithm>

#include <quadrotor/quadrotor.h>
#include <quadrotor/telem.h>
#include <quadrotor/plant.h>

Plant::Plant(Quadrotor* quad):
	quad_(quad)
{
	int n = quad_->N_;
		
	// constants

	// motor speed
	gamma0_.alloc(n);
	gamma1_.alloc(n);

	gamma0_act_.alloc(n);
	gamma1_act_.alloc(n);


	tau_RB_.alloc(n);
	f_RB_.alloc(n);

}
math::vec3 Plant::get_tau_body(int ti) {
	math::vec3 tau = tau_RB_[ti];
	return tau;
}
void Plant::step_rotor_body(int ti) {
	math::vec4 gamma1 = gamma1_[ti];
	double gamma0 = gamma0_[ti];

	//if (gamma.isNan()) throw;
	//raise ValueError('gamma nan')
	
	math::vec4 gamma = gamma1 + gamma0;

	double g_max = std::max(
			std::max(fabs(gamma.x), fabs(gamma.y)),
			std::max(fabs(gamma.z), fabs(gamma.w)));
	
	double g1_max = std::max(
			std::max(gamma1.x, gamma1.y),
			std::max(gamma1.z, gamma1.w));
	double g1_min = std::min(
			std::min(gamma1.x, gamma1.y),
			std::min(gamma1.z, gamma1.w));
	
	double g1_abs_max = std::max(fabs(g1_min), fabs(g1_max));
	
	// in the event that the reference gamma is too high
	// torque is a priority, followed by thrust
	if(g_max > quad_->gamma_max_) {
		if(g1_abs_max < quad_->gamma_max_) { // torque can be satisfied if thrust is reduced
			double g0;

			// product as must thrust as allowed
			if(gamma0 > 0.0) { // g1_max is the limiter
				g0 = quad_->gamma_max_ - g1_max;
			} else { // g1_min is the limiter
				g0 = -quad_->gamma_max_ - g1_min;
			}

			gamma0 = g0;
		} else {
			gamma0 = 0.0;
			
			// scale gamma1
			gamma1 *= quad_->gamma_max_ / g1_abs_max;
		}
	}

	//record
	gamma0_act_[ti] = gamma0;
	gamma1_act_[ti] = gamma1;

	
	math::vec4 temp = quad_->A4_ * gamma1;

	// torque
	math::vec3 tau(temp.x, temp.y, temp.z);
	tau_RB_[ti] = tau;
	
	// force
	double thrust = 4.0 * quad_->k_ * gamma0;
	math::vec3 T(0.0, 0.0, thrust);

	f_RB_[ti] = T;
	
	//printf("f_RB\n");
	//f_RB_[ti].print();
/*
	if(tau.isNan()) {
		printf("A4\n");
		quad_->A4_.print();
		printf("gamma\n");
		gamma1.print();
		throw;
	}
	if(!f_RB_[ti].isSane()) {
		printf("gamma0 %f\n", gamma0);
		printf("f_RB\n");
		f_RB_[ti].print();
		throw;
	}*/
}
math::vec3 Plant::get_force_drag_body(int ti) {
	return math::vec3();
}
math::vec3 Plant::get_force_drag(int ti) {
	return quad_->telem_->q_[ti].getConjugate().rotate(get_force_drag_body(ti));
}
math::vec3 Plant::get_force(int ti) {
	
	math::vec3 f_B = f_RB_[ti] + get_force_drag_body(ti);
	
	//if (f_B.isNan()) raise ValueError("f_B nan");

	math::vec3 f = quad_->telem_->q_[ti].getConjugate().rotate(f_B);
	
	

	/*
	   ver = False
	   if ver:	
	   print 'A6 ',A6
	   print 'f_g',f_g
	   print 'f_B',f_B
	   print 'f  ',f
	   */
	return f;
}
void Plant::step(int i) {
	//double dt = t_[ti] - t_[ti-1];
	
	step_rotor_body(i);
	
	// rotation
	math::vec3 tau = tau_RB_[i];
	
	quad_->alpha(i) = quad_->Iinv_ * (tau - quad_->omega(i-1).cross(quad_->I_ * quad_->omega(i-1)));
	
	// translation
	math::vec3 f = get_force(i);
	
	quad_->a(i) = quad_->gravity_ + f / quad_->m_;
}
void Plant::write(int n) {
	FILE* file = fopen("data/plant.txt","w");

	n = (n > 0) ? (n) : (quad_->N_);

	gamma1_.write(file, n);
	gamma1_act_.write(file, n);

	tau_RB_.write(file, n);
	f_RB_.write(file, n);

	gamma0_.write(file, n);
	gamma0_act_.write(file, n);

	fclose(file);
}


