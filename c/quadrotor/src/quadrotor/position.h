#ifndef __POSITION__
#define __POSITION__

#include <math/mat33.h>

#include <quadrotor/brain.h>
#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/telem.h>
#include <quadrotor/plant.h>

class Position {
	public:
		Position(Quadrotor*);
		void		reset();
		void		fill_xref(int ti1, math::vec3 x);
		void		fill_xref_parametric(int ti1, math::vec3 (*f)(double));

		void		set_poles(double*);

		void		step(double dt, int ti, int ti_0);

		void		step_accel(double dt, int ti, int ti_0);
		void		step_jerk(double dt, int ti, int ti_0);
		void		step_jounce(double dt, int ti, int ti_0);

		void		check_command(int);

		//void		set_obj(int ti, Command::Position* pos);

		//void		get_force_rotor(int ti, int ti_0);

		void		write(int n = 0);
		void		write_param();
		void		read_param();

	public:
		Quadrotor*	quad_;

		//Command::Position*	pos_;

		math::mat33		C1_;
		math::mat33		C2_;
		math::mat33		C3_;
		math::mat33		C4_;
		math::mat33		C5_;

		Array<math::vec3>	e1_;
		Array<math::vec3>	e2_;
		Array<math::vec3>	e3_;
		Array<math::vec3>	e4_;

		Array<math::vec3>	chi_;

		Array<double>		e1_mag_;
		Array<double>		e1_mag_d_;
		Array<double>		e1_mag_dd_;
		
		Array<math::vec3>	x_ref_;
		Array<math::vec3>	x_ref_d_;
		Array<math::vec3>	x_ref_dd_;
		Array<math::vec3>	x_ref_ddd_;
		Array<math::vec3>	x_ref_dddd_;

		Array<math::vec3>	a_;
		Array<math::vec3>	jerk_;
		Array<math::vec3>	jounce_;

		
		
		unsigned int	flag_;
};

#endif


