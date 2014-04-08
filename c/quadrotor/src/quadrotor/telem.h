#ifndef __TELEM__
#define __TELEM__

#include <stdio.h>

#include <math/vec3.h>
#include <math/vec4.h>
#include <math/quat.h>
#include <math/mat33.h>
#include <math/mat44.h>

#include <quadrotor/array.h>
#include <quadrotor/except.h>

class Quadrotor;

class Telem {
	public:

	public:
		Telem(Quadrotor*);

		void		step(int ti);

		void		write(int ti);
	public:
		Quadrotor*	quad_;

		// state variables
		Array<math::quat>	q_;
		Array<math::vec3>	o_;
	//	Array<math::vec3>	al_;
		
		Array<math::vec3>	x_;
		Array<math::vec3>	v_;
	//	Array<math::vec3>	a_;

};


#endif



