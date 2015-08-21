#include <math/math.hpp>

#include <drone/util/print.hpp>
#include <drone/util/check.hpp>
#include <drone/Drone.hpp>
#include <drone/command/command.h>
#include <drone/command/Input.hpp>
#include <drone/except.h>
#include <drone/fda.h>
#include <drone/cl/ControlLaw.h>

#include <drone/cl/Snap.hpp>

void	Jounce::Base::step(int i, float h)
{
	auto drone = get_drone();

	// rotate jounce*mass vector to body frame?
	// second derivative of rotor force in the body frame
	glm::vec3 F_RB_2 = drone->q(i) * (jounce_[i] * drone->m_);
	
	DRONE_CHECK4(F_RB_2, drone->q(i), jounce_[i], drone->m_);
	
	glm::vec3& o = drone->omega(i);

	// thrust
	float h2 = h*h;
	//
	float temp = F_RB_2.z - (thrust_[i-2] - 2.0 * thrust_[i-1]) / h2;
	
	thrust_[i] = temp / (1.0 / h2 - o.x * o.x - o.y * o.y);
	

	/*
	// limit thrust
	float thrust_limit = 1e6;
	if(thrust_[i] > thrust_limit) {
		thrust_[i] = thrust_limit;
	} else if(thrust_[i] < -thrust_limit) {
		thrust_[i] = -thrust_limit;
	}
	*/

	// estimate first derivative of thrust
	//float thrust_d = (thrust_[i] - thrust_[i-1]) / h;
	float thrust_d = (3*thrust_[i] - 4*thrust_[i-1] + thrust_[i-2]) / (2*h);

	
	DRONE_CHECK4(thrust_[i], F_RB_2, h, o);
	
	// angular acceleration
	if(thrust_[i] != 0) {
		alpha_[i].y =  (thrust_[i] * o.x * o.z - 2.0 * thrust_d * o.y - F_RB_2.x)/thrust_[i];
		alpha_[i].x = -(thrust_[i] * o.y * o.z + 2.0 * thrust_d * o.x - F_RB_2.y)/thrust_[i];
	} else {
		alpha_[i] = glm::vec3(0);
	}
	
	if(drone->isset_debug()) {
	printf("\n");
	printf("thrust  %16e\n", thrust_[i]);
	printf("jounce(%i)  %16e%16e%16e\n", i, jounce_[i].x, jounce_[i].y, jounce_[i].z);
	printf("alpha(%i)   %16e%16e%16e\n", i, alpha_[i].x, alpha_[i].y, alpha_[i].z);
	printf("omega(%i)   %16e%16e%16e\n", i, o.x, o.y, o.z);
	}

	CL::Thrust::step(i,h);
	CL::Alpha::step(i,h);
	
}
void	Jounce::Base::alloc(int n)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	CL::Thrust::alloc(n);
	CL::Alpha::alloc(n);
	jounce_.alloc(n);
}
void	Jounce::Base::write(std::string s, int n)
{
	std::string f = "data/"+s+".txt";
	//printf("%s\n",__PRETTY_FUNCTION__);
	FILE* file = fopen(f.c_str(),"w");
	if(!file) {
		perror("fopen");
		throw;
	}
	jounce_.write(file,n);
	fclose(file);

	CL::Thrust::write("thrust_x",n);
	CL::Alpha::write("alpha_x",n);
}



