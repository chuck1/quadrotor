
#include <quadrotor/ControlLaw/Alpha.h>

void	Alpha1::Base::alloc(int n) {
	CL::Alpha::alloc(n);
}
void	Alpha1::Q::alloc(int n) {
	Alpha1::Base::alloc(n);
	CL::Q<2>::alloc(n);
}
void	Alpha1::Omega::Step(int i, double h) {

	alpha_[i] = 
		c_[0] * e_[0][i] +
		c_[1] * e_[1][i] +
		omega_ref_[1][i];
}
void	Alpha1::Omega::alloc(int n) {
	Alpha1::Base::alloc(n);
	CL::Omega<2>::alloc(n);
}

