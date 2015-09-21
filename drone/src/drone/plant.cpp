
#include <stdio.h>

#include <algorithm>

#include <glm/gtc/quaternion.hpp>

#include <math/math.hpp>

#include <drone/Drone.hpp>
#include <drone/Telem.hpp>
#include <drone/Plant.hpp>

Plant::Plant(std::shared_ptr<Drone> drone):
	quad_(drone)
{
	int n = quad_.lock()->N_;
		
	// constants

	// motor speed
	//gamma0_.alloc(n);
	//gamma1_.alloc(n);

	gamma0_act_.alloc(n);
	gamma1_act_.alloc(n);


	tau_RB_.alloc(n);
	f_RB_.alloc(n);

}
glm::vec3 Plant::get_tau_body(int ti)
{
	glm::vec3 tau = tau_RB_[ti];
	return tau;
}
void Plant::step_rotor_body(int ti)
{
	auto drone = get_drone();

	float		gamma0 = drone->gamma0_[ti];
	glm::vec4	gamma1 = drone->gamma1_[ti];

	if(isnan(gamma0) || isinf(gamma0)) throw 0;
	if(math::is_nan_or_inf(gamma1)) throw 0;
	//raise ValueError('gamma nan')
	
	glm::vec4 gamma = gamma1 + gamma0;

	float g_max = std::max(
			std::max(fabs(gamma.x), fabs(gamma.y)),
			std::max(fabs(gamma.z), fabs(gamma.w)));
	
	float g1_max = std::max(
			std::max(gamma1.x, gamma1.y),
			std::max(gamma1.z, gamma1.w));
	float g1_min = std::min(
			std::min(gamma1.x, gamma1.y),
			std::min(gamma1.z, gamma1.w));
	
	float g1_abs_max = std::max(fabs(g1_min), fabs(g1_max));
	
	// in the event that the reference gamma is too high
	// torque is a priority, followed by thrust
	if(g_max > drone->gamma_max_) {
		if(g1_abs_max < drone->gamma_max_) { // torque can be satisfied if thrust is reduced
			float g0;

			// product as must thrust as allowed
			if(gamma0 > 0.0) { // g1_max is the limiter
				g0 = drone->gamma_max_ - g1_max;
			} else { // g1_min is the limiter
				g0 = -drone->gamma_max_ - g1_min;
			}

			gamma0 = g0;
		} else {
			gamma0 = 0.0;
			
			// scale gamma1
			gamma1 *= drone->gamma_max_ / g1_abs_max;
		}
	}

	//record
	gamma0_act_[ti] = gamma0;
	gamma1_act_[ti] = gamma1;

	
	glm::vec4 temp = drone->A4_ * gamma1;

	// torque
	glm::vec3 tau(temp.x, temp.y, temp.z);
	tau_RB_[ti] = tau;
	
	// force
	float thrust = 4.0 * drone->k_ * gamma0;
	glm::vec3 T(0.0, 0.0, thrust);

	f_RB_[ti] = T;
	
	//printf("f_RB\n");
	//f_RB_[ti].print();
	
	if(math::is_nan_or_inf(tau)) {
		printf("A4\n");
		//quad_->A4_.print();
		printf("gamma\n");
		//gamma1.print();
		throw 0;
	}
	if(math::is_nan_or_inf(f_RB_[ti])) {
		printf("gamma0 %f\n", gamma0);
		printf("f_RB\n");
		//f_RB_[ti].print();
		throw 0;
	}
}
glm::vec3 Plant::get_force_drag_body(int ti)
{
	return glm::vec3();
}
glm::vec3 Plant::get_force_drag(int ti)
{
	auto drone = get_drone();
	return glm::conjugate(drone->telem_->q_[ti]) * get_force_drag_body(ti);
}
glm::vec3 Plant::get_force(int ti)
{
	auto drone = get_drone();
	
	glm::vec3 f_B = f_RB_[ti] + get_force_drag_body(ti);
	
	if(math::is_nan_or_inf(f_B)) {
		throw 0;
	}

	glm::vec3 f = glm::conjugate(drone->telem_->q_[ti]) * f_B;

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
void Plant::step(int i)
{
	auto drone = get_drone();
	//float dt = t_[ti] - t_[ti-1];
	
	step_rotor_body(i);
	
	// rotation
	glm::vec3 & t = tau_RB_[i];
	
	// really need to formalize when to use what time step values
	glm::vec3 & o = drone->omega(i);
	//glm::vec3 & o = quad_->omega(i-1);

	glm::mat3 & I = drone->I_;
	
	glm::vec3 a = drone->Iinv_ * (t - glm::cross(o, I * o));

	drone->alpha(i) = a;

	if(drone->isset_debug()) {
	printf("plant\n");
	printf("  torque(i) %16e%16e%16e\n", t.x, t.y, t.z);
	printf("  omega(i)  %16e%16e%16e\n", o.x, o.y, o.z);
	printf("  alpha(i)  %16e%16e%16e\n", a.x, a.y, a.z);
	}

	// translation
	glm::vec3 f = get_force(i);
	
	drone->a(i) = drone->gravity_ + f / drone->m_;
}
void Plant::write(int n)
{
	auto drone = get_drone();

	FILE* file = fopen("data/plant.txt","w");

	n = (n > 0) ? (n) : (drone->N_);

	//gamma1_.write(file, n);
	gamma1_act_.write(file, n);

	tau_RB_.write(file, n);
	f_RB_.write(file, n);

	//gamma0_.write(file, n);
	gamma0_act_.write(file, n);

	fclose(file);
}


