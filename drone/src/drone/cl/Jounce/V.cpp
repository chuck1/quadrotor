#include <math/math.hpp>

#include <drone/util/print.hpp>
#include <drone/Drone.hpp>
#include <drone/command/command.h>
#include <drone/except.h>
#include <drone/fda.h>
#include <drone/command/Input.hpp>
#include <drone/cl/ControlLaw.h>

#include <drone/cl/Snap.hpp>

/*
Jounce::V::V(Quadrotor* r): CL::Base(r), CL::V<4>(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {
	printf("%s\n",__PRETTY_FUNCTION__);
	alloc(r->N_);
}
*/
void	Jounce::V::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	
	auto drone = get_drone();

	//assert(command_);
	auto v = std::dynamic_pointer_cast<Command::V>(get_command());
	assert(v);
	
	if(i == 0) {
		// back fill
		v_ref_[0][-1] = v->in_->f(drone->t(-1));
		v_ref_[0][-2] = v->in_->f(drone->t(-2));
		v_ref_[0][-3] = v->in_->f(drone->t(-3));
	}

	v_ref_[0][i] = v->in_->f(drone->t(i));
	//v_ref_[0][i].print();

	// reference derivatives

	forward(v_ref_[0], v_ref_[1], h, i);
	forward(v_ref_[1], v_ref_[2], h, i);
	forward(v_ref_[2], v_ref_[3], h, i);


	e_[1][i] = v_ref_[0][i] - drone->v(i);
	e_[2][i] = v_ref_[1][i] - drone->a(i);
	e_[3][i] = v_ref_[2][i] - drone->jerk(i);

	// integral
	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	//c_[1].print();

	jounce_[i] = 
		c_[0] * e_[0][i] + 
		c_[1] * e_[1][i] + 
		c_[2] * e_[2][i] + 
		c_[3] * e_[3][i] +
		v_ref_[3][i];

	//jounce_[i].print();

	Jounce::Base::step(i, h);
}
bool	Jounce::V::check(int i, glm::vec3 tol)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	if (glm::all(glm::lessThan(glm::abs(e_[1][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(e_[2][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(e_[3][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(jounce_[i-1]), tol))) {

					printf("i=%i\n",i);
					//e_[1][i].print();
					//e_[2][i].print();
					//e_[3][i].print();
					return true;
					//command->Settle(i, r_->t_[i]);
				}
			}
		}
	}
	return false;
}
void	Jounce::V::alloc(int n)
{
	Jounce::Base::alloc(n);
	CL::V<4>::alloc(n);
}
void	Jounce::V::write(int n)
{
	Jounce::Base::write("snap_v", n);
	CL::V<4>::write(n);
}

