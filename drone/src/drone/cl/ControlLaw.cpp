
#include <drone/util/check.hpp>
#include <drone/Plant.hpp>
#include <drone/Drone.hpp>
#include <drone/cl/ControlLaw.h>

CL::Base::Base()
{
}
void	CL::Base::init()
{
	assert(r_);
	alloc(r_->N_);
}
void	CL::Thrust::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	
	auto g = thrust_[i] / (r_->k_ * 4.0);

	drone::util::check(r_, __FILE__, __LINE__, g, thrust_[i], r_->k_);

	r_->plant_->gamma0_[i] = g;
}
void	CL::Alpha::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	glm::vec3 torque = r_->angular_accel_to_torque(i, alpha_[i]);
	
	// TODO
	//glm::vec4 temp(0,torque);
	glm::vec4 temp(torque,0);
	
	auto g = r_->A4inv_ * temp;
	
	drone::util::check(r_, __FILE__, __LINE__, g, r_->A4inv_, temp);

	r_->plant_->gamma1_[i] = g;
}
void	CL::Alpha::alloc(int n)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	alpha_.alloc(n);
}
void	CL::Alpha::write(int n)
{
	FILE* file = fopen("data/alpha.txt","w");
	alpha_.write(file,n);
	fclose(file);
}
void	CL::Thrust::alloc(int n)
{
	thrust_.alloc(n);

	float t = r_->m_ * -r_->gravity_.z;
	thrust_[-1] = t;
	thrust_[-2] = t;
}
void	CL::Thrust::write(int n)
{
	FILE* file = fopen("data/thrust.txt","w");
	thrust_.write(file,n);
	fclose(file);
}
float coeff(float* r, int n, int i, int k)
{
	// coeff -- coefficiencts to a polynomial with n roots stored in r
	// i must start at 0
	// k is the power of x associated with this coeff
	// c[k] == 1
	// do not call this function with k==n
	
	//int i__ = i;

	float c = 0;

	for(; i <= k; i++) {
		//printf("%i %i\n",i,k);
		if((k+1) < n) {
			//printf("descend\n");
			c += r[i] * coeff(r, n, i+1, k+1);
		} else {
			//printf("stop\n");
			c += r[i];
			//printf("%e %e %i\n",c,r[i],i);
		}
	}

	//if(i__==0) printf("%e\n",-c);

	return -c;
}


