#ifndef __BRAIN__
#define __BRAIN__

#include <deque>

#include <math/quat.h>
#include <math/vec4.h>

#include <quadrotor/array.h>
#include <quadrotor/except.h>

class Position;
class Attitude;
class Quadrotor;

namespace CL {
	class Base;
	template<int> class X;
	template<int> class V;
	template<int> class Q;
}

namespace Command {
	class Base;
}

class Brain {
	public:
		struct Mode {
			enum e {
				NONE = 0,
				ACCEL,
				JERK,
				JOUNCE
			};
		};

	public:
		Brain(Quadrotor*);

		void		reset();
		
		void		CheckCommand(int);
		void		step(int ti, double dt);
	
		void		write(int ti);

	public:
		int		mode_;

		Quadrotor*	quad_;

		// where the magic happends
		CL::Base*	cl_;
		
		CL::Base*	cl_x_;
		CL::Base*	cl_v_;
		CL::Base*	cl_q_;

		//Position*	pos_;
		//Attitude*	att_;
		

		//Array<math::vec3>	f_R_;
		//Array<math::vec3>	tau_R_;
		
		//Array<math::vec3>	a_RB_;
		
		//Array<double>		thrust_;
		//Array<double>		thrust_d_;
		
		double		heading_;

		std::deque<Command::Base*>	objs_;
		
		

		Command::Base*	obj_;
};



#endif
