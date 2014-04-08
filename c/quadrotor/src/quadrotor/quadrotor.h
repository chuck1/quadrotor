#ifndef __QUADROTOR__
#define __QUADROTOR__

#include <math/vec3.h>
#include <math/mat33.h>
#include <math/mat44.h>

class Telem;
class Plant;
class Brain;

void product(int choices, int repeat, int*& arr, int level = 0);


class Quadrotor {
	public:
		Quadrotor(double dt, int N);
		void	reset();
		void	run();
		void	write();
		void	write_param();
	public:
		// physical constants
		double		m_, L_, R_, Asw_, rho_, CD_, A_;
		double		Kv_, Kt_, Ktau_;
		double		k_, b_;
		double		P_max_, gamma_max_;

		math::mat33	I_;
		math::mat33	Iinv_;

		math::vec3	gravity_;

		math::mat44	A4_;
		math::mat44	A4inv_;

		

		double		dt_;
		int		N_;
		double*		t_;
		
		// stop criteria
		int		ti_stop_;
		
		int		ti_f_;
		
		Telem*		telem_;
		Plant*		plant_;
		Brain*		brain_;

};


#endif





