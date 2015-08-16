#ifndef __POSITION__
#define __POSITION__

#include <glm/glm.hpp>

#include <drone/brain.h>
#include <drone/command.h>
#include <drone/quadrotor.h>
#include <drone/telem.h>
#include <drone/plant.h>

class Position {
	public:
		Position(Quadrotor*);
		void		reset();
		void		fill_xref(int ti1, glm::vec3 x);
		void		fill_xref_parametric(int ti1, glm::vec3 (*f)(double));

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

		glm::mat3		C1_;
		glm::mat3		C2_;
		glm::mat3		C3_;
		glm::mat3		C4_;
		glm::mat3		C5_;

		Array<glm::vec3>	e1_;
		Array<glm::vec3>	e2_;
		Array<glm::vec3>	e3_;
		Array<glm::vec3>	e4_;

		Array<glm::vec3>	chi_;

		Array<double>		e1_mag_;
		Array<double>		e1_mag_d_;
		Array<double>		e1_mag_dd_;
		
		Array<glm::vec3>	x_ref_;
		Array<glm::vec3>	x_ref_d_;
		Array<glm::vec3>	x_ref_dd_;
		Array<glm::vec3>	x_ref_ddd_;
		Array<glm::vec3>	x_ref_dddd_;

		Array<glm::vec3>	a_;
		Array<glm::vec3>	jerk_;
		Array<glm::vec3>	jounce_;

		
		
		unsigned int	flag_;
};

#endif


