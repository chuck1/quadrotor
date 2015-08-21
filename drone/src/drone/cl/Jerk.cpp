
#include <drone/fda.h>
#include <drone/cl/ControlLaw.h>
#include <drone/cl/Alpha.hpp>
#include <drone/cl/Jerk.hpp>

//: Alpha::Omega(r), CL::Thrust(r) {}
//, CL::Alpha(r) {}
void	Jerk::X::step(int i, float h)
{
	auto drone = get_drone();

	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);


	CL::X<4>::e_[1][i] = drone->x(i) - x_ref_[0][i];
	CL::X<4>::e_[2][i] = drone->v(i) - x_ref_[1][i];
	CL::X<4>::e_[3][i] = drone->a(i) - x_ref_[2][i];

	CL::X<4>::e_[0][i] = CL::X<4>::e_[0][i-1] + CL::X<4>::e_[1][i] * h;



	jerk_[i] = 
		CL::X<4>::c_[0] * CL::X<4>::e_[0][i] + 
		CL::X<4>::c_[1] * CL::X<4>::e_[1][i] + 
		CL::X<4>::c_[2] * CL::X<4>::e_[2][i] + 
		CL::X<4>::c_[3] * CL::X<4>::e_[3][i] +
		x_ref_[3][i];
	
	glm::vec3 tmp = drone->q(i) * (jerk_[i] * drone->m_);
	
	thrust_[i] = thrust_[i-1] + tmp.z * h;

	glm::vec3 o;
	if(thrust_[i] > 0) {
		o.x =  tmp.y / thrust_[i];
		o.y = -tmp.x / thrust_[i];
	}

/*
	if(!o.IsSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}
*/
	//set_ref(i, o);
	omega_ref_[0][i] = o;

}


