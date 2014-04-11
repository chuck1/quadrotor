#ifndef __PLANT__
#define __PLANT__

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat33.h>
#include <math/mat44.h>

#include <quadrotor/array.h>

class Quadrotor;

class Plant {
	public:
		Plant(Quadrotor* quad);

		math::vec3	get_tau_body(int ti);
		void		step_rotor_body(int ti);
		
		math::vec3	get_force_rotor_body(int ti);

		math::vec3	get_force_drag_body(int ti);
		math::vec3	get_force_drag(int ti);

		math::vec3	get_force(int ti);

		void		step(int ti);

		void		write(int n);

	public:
		Quadrotor*	quad_;

		// state variables

		Array<double>		gamma0_;
		Array<math::vec4>	gamma1_;
	
		Array<double>		gamma0_act_;
		Array<math::vec4>	gamma1_act_;
	
		Array<math::vec3>	tau_RB_;
		
		Array<math::vec3>	f_RB_;
		
		

};




#endif
