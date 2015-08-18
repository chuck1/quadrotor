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
	// rotate jounce*mass vector to body frame?
	glm::vec3 tmp = r_->q(i) * (jounce_[i] * r_->m_);
	
	DRONE_CHECK(tmp, r_->q(i), jounce_[i], r_->m_);
	
	glm::vec3& o = r_->omega(i);

	// thrust
	float temp = tmp.z - (thrust_[i-2] - 2.0 * thrust_[i-1]) / h / h;

	thrust_[i] = temp / (1.0 / h / h - o.x * o.x - o.y * o.y);

	// estimate first derivative of thrust
	//float thrust_d = (thrust_[i] - thrust_[i-1]) / h;
	float thrust_d = (3*thrust_[i] - 4*thrust_[i-1] + thrust_[i-2]) / (2*h);
	
	DRONE_CHECK(thrust_[i], tmp, h, o);
	
	// angular acceleration
	if(thrust_[i] != 0) {
		alpha_[i].y =  (thrust_[i] * o.x * o.z - 2.0 * thrust_d * o.y - tmp.x) / thrust_[i];
		alpha_[i].x = -(thrust_[i] * o.y * o.z + 2.0 * thrust_d * o.x - tmp.y) / thrust_[i];
	} else {
		alpha_[i] = glm::vec3(0);
	}
	
	printf("\n");
	printf("thrust  %16e\n", thrust_[i]);
	printf("jounce(%i)  %16e%16e%16e\n", i, jounce_[i].x, jounce_[i].y, jounce_[i].z);
	printf("alpha(%i)   %16e%16e%16e\n", i, alpha_[i].x, alpha_[i].y, alpha_[i].z);
	printf("omega(%i)   %16e%16e%16e\n", i, o.x, o.y, o.z);

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
void	Jounce::Base::write(int n)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	FILE* file = fopen("data/jounce.txt","w");
	if(!file) {
		perror("fopen");
		throw;
	}
	jounce_.write(file,n);
	fclose(file);

	CL::Thrust::write(n);
	CL::Alpha::write(n);
}



