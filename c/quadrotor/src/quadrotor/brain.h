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

namespace Command {
	class Base;
}

class Brain {
	public:
		struct Mode {
			enum e {
				ACCEL,
				IMPULSE,
				NONE
			};
		};

	public:
		Brain(Quadrotor*);

		void		reset();

		//void		process_force_reference(int ti);
		
		void		control_law_position(double dt, int ti, int ti_0);
		void		control_law_3(double dt, int ti, int ti_0);
		
		void		step(double dt, int ti);
	
		void		step_accel(int ti);
		void		step_impulse(double dt, int ti);

		void		step_motor_speed(int ti);

		void		write(int ti);

	public:
		int		mode_;

		int		ti_0_;
		
		Quadrotor*	quad_;

		Position*	pos_;
		Attitude*	att_;
		

		Array<math::vec3>	f_R_;
		Array<math::vec3>	tau_R_;

		Array<math::vec3>	a_RB_;

		Array<double>		thrust_;

		double		heading_;

		std::deque<Command::Base*>	objs_;

		Command::Base*	obj_;
};



#endif
