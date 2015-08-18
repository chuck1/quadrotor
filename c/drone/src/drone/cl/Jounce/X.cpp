#include <math/math.hpp>

#include <drone/Drone.hpp>
#include <drone/command/command.h>
#include <drone/except.h>
#include <drone/fda.h>
#include <drone/command/Input.hpp>
#include <drone/cl/ControlLaw.h>
#include <drone/cl/Snap.hpp>

/*
Jounce::X::X(): CL::Base(r), CL::X<5>(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {
	printf("%s\n",__PRETTY_FUNCTION__);
	alloc(r->N_);

}
*/
bool	Jounce::X::check(int i, glm::vec3 tol)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	//Command::X* command = (Command::X*)command_;

	if (glm::all(glm::lessThan(glm::abs(e_[1][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(e_[1][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(e_[1][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(e_[1][i]), tol))) {
	if (glm::all(glm::lessThan(glm::abs(jounce_[i-1]), tol))) {
		//command->Settle(i, r_->t_[i]);
		return true;
	}}}}}
	
	return false;
}
void	Jounce::X::step(int i, float h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	//printf("%f\n",h);

	Command::X* x = dynamic_cast<Command::X*>(command_);
	assert(x);

	if(i == 0) {	
		// back fill
		x_ref_[0][-1] = x->in_->f(r_->t(-1));
		x_ref_[0][-2] = x->in_->f(r_->t(-2));
		x_ref_[0][-3] = x->in_->f(r_->t(-3));
	}

	x_ref_[0][i] = x->in_->f(r_->t(i));

	forward(x_ref_[0], x_ref_[1], h, i);
	forward(x_ref_[1], x_ref_[2], h, i);
	forward(x_ref_[2], x_ref_[3], h, i);
	forward(x_ref_[3], x_ref_[4], h, i);

	e_[1][i] = x_ref_[0][i] - r_->x(i);
	e_[2][i] = x_ref_[1][i] - r_->v(i);
	e_[3][i] = x_ref_[2][i] - r_->a(i-1);
	e_[4][i] = x_ref_[3][i] - r_->jerk(i-1);

	//printf("e1\n");
	//print(e_[1][i]);

	// integral
	e_[0][i] = e_[0][i-1] + e_[1][i] * h;

	jounce_[i] = 
		c_[0] * e_[0][i] + 
		c_[1] * e_[1][i] + 
		c_[2] * e_[2][i] + 
		c_[3] * e_[3][i] +
		c_[4] * e_[4][i] +
		x_ref_[4][i];

	Jounce::Base::step(i, h);

#if 0
	if(0) {
		printf("e_[2][i]\n");
		e_[2][i].print();
		x_ref_[1][i].print();
		r_->v(i).print();
		printf("x_ref_[0][%i]\n",i);
		x_ref_[0][i].print();
		printf("x_ref_[0][%i]\n",i-1);
		x_ref_[0][i-1].print();
	}
#endif
}
void	Jounce::X::alloc(int n)
{
	//printf("%s\n",__PRETTY_FUNCTION__);
	Jounce::Base::alloc(n);
	CL::X<5>::alloc(n);
}
void	Jounce::X::write(int n) {
	Jounce::Base::write(n);
	CL::X<5>::write(n);
}

