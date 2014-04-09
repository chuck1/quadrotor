#ifndef __ATTITUDE__
#define __ATTITUDE__

#include <math/vec3.h>
#include <math/quat.h>
#include <math/mat33.h>

#include <quadrotor/array.h>

class Quadrotor;

namespace Command {
	class Orient;
}

class Attitude {
	public:
		struct Mode {
			enum e {
				ATT,
				VEL,
				NONE
			};
		};

	public:
		Attitude(Quadrotor* quad);
		void		reset();

		void		set_q_reference(int ti, math::quat q);
		void		set_o_reference(int ti, math::vec3 o);
		void		set_o_reference(int ti, double, double, double);

		

		void		set_obj(int ti1, Command::Orient* att);

		void		step(double dt, int ti, int ti_0);
		
		void		step_torque_rotor_body(int ti, int ti_0);
		void		step_torque_rotor_body_att(int ti, int ti_0);
		void		step_torque_rotor_body_vel(int ti, int ti_0);
		void		step_torque_rotor_body(int, math::vec3);


		void		write(int n = 0);
		void		write_param();
		void		read_param();

	public:
		unsigned int			mode_;
		
		Quadrotor*			quad_;
		Command::Orient*		att_;	
		
		math::mat33			C1_;
		math::mat33			C2_;
		
		Array<math::quat>		e1_;
		Array<math::vec3>		e2_;
		
		Array<math::quat>		q_ref_;
		Array<math::vec3>		q_ref_d_;
		Array<math::vec3>		q_ref_dd_;
		
		Array<math::vec3>		o_ref_;
		Array<math::vec3>		o_ref_d_;

		Array<double>			e1_mag_;
		Array<double>			e1_mag_d_;
		
		Array<math::vec3>		tau_RB_;



};

#endif
