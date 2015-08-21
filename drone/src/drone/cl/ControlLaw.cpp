
#include <drone/util/check.hpp>
#include <drone/Plant.hpp>
#include <drone/Brain.hpp>
#include <drone/Drone.hpp>
#include <drone/cl/ControlLaw.h>

CL::Base::Base()
{
}
void	CL::Base::init()
{
	alloc(get_drone()->N_);
}
std::shared_ptr<Quadrotor>	CL::Base::get_drone()
{
	auto drone = _M_drone.lock();
	assert(drone);
	return drone;
}
std::shared_ptr<Command::Base>	CL::Base::get_command()
{
	auto b = get_drone()->brain_;
	assert(b);

	return b->get_obj();
}
void	CL::Thrust::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	auto drone = get_drone();

	auto g = thrust_[i] / (drone->k_ * 4.0);

	drone::util::check(drone.get(), __FILE__, __LINE__, g, thrust_[i], drone->k_);

	drone->plant_->gamma0_[i] = g;
}
void	CL::Alpha::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	auto drone = get_drone();

	glm::vec3 torque = drone->angular_accel_to_torque(i, alpha_[i]);
	
	// TODO
	//glm::vec4 temp(0,torque);
	glm::vec4 temp(torque,0);
	
	auto g = drone->A4inv_ * temp;
	
	drone::util::check(drone.get(), __FILE__, __LINE__, g, drone->A4inv_, temp);

	drone->plant_->gamma1_[i] = g;
}
void	CL::Alpha::alloc(int n)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	alpha_.alloc(n);
}
void	CL::Alpha::write(std::string s, int n)
{
	std::string f = "data/"+s+".txt";
	FILE* file = fopen(f.c_str(),"w");
	alpha_.write(file,n);
	fclose(file);
}
void	CL::Thrust::alloc(int n)
{
	thrust_.alloc(n);

	auto drone = get_drone();

	float t = drone->m_ * -drone->gravity_.z;
	thrust_[-1] = t;
	thrust_[-2] = t;
}
void	CL::Thrust::write(std::string s, int n)
{
	std::string f = "data/"+s+".txt";
	FILE* file = fopen(f.c_str(),"w");
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


