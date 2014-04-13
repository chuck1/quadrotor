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
		void		reset();
		void		run();
		
		math::vec3	angular_accel_to_torque(int, math::vec3);
		math::vec4	thrust_torque_to_motor_speed(int, double const &, math::vec3 const &);
		
		void		write();
		
		void		write_param();
		
		// accessors
		math::vec3&	x(int);
		math::vec3&	v(int);
		math::vec3&	a(int);
		math::vec3&	jerk(int);
		//math::vec3&	jounce(int);
		math::quat&	q(int);
		math::vec3&	omega(int);
		math::vec3&	alpha(int);
		
		double		t(int i) const { return dt_ * (double)i; }
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





