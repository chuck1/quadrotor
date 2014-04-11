
#include <quadrotor/ControlLaw/Alpha.h>

void	Alpha1::Omega::Step(int i, double h) {

	alpha_[i] = 
		c_[0] * e_[0][i] +
		c_[1] * e_[1][i] +
		omega_ref_[1][i];
}

