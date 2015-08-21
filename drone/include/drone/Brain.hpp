#ifndef __BRAIN__
#define __BRAIN__

#include <deque>
#include <memory>

#include <glm/glm.hpp>

#include <drone/array.h>
#include <drone/except.h>

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
		Brain(std::shared_ptr<Quadrotor>);

		void		reset();
		
		void		CheckCommand(int);
		void		step(int ti, double dt);
	
		void		write(int ti);

		std::shared_ptr<Command::Base>		get_obj();

		std::shared_ptr<CL::Base>		get_cl();
	public:
		int					mode_;

		std::weak_ptr<Quadrotor>		_M_drone;

		// where the magic happends
		//std::shared_ptr<CL::Base>	cl_;
		
		std::shared_ptr<CL::Base>		cl_x_;
		std::shared_ptr<CL::Base>		cl_v_;
		std::shared_ptr<CL::Base>		cl_q_;

		//Position*	pos_;
		//Attitude*	att_;
		

		//Array<math::vec3>	f_R_;
		//Array<math::vec3>	tau_R_;
		
		//Array<math::vec3>	a_RB_;
		
		//Array<double>		thrust_;
		//Array<double>		thrust_d_;
		
		double		heading_;

		std::deque< std::shared_ptr<Command::Base> >	objs_;

		//std::shared_ptr<Command::Base>			obj_;
};



#endif
