#ifndef __QUADROTOR__
#define __QUADROTOR__

#include <glm/glm.hpp>

#include <drone/array.h>

class Telem;
class Plant;
class Brain;

void product(int choices, int repeat, int*& arr, int level = 0);


class Quadrotor
{
	public:
		enum StopCause
		{
			INF = 0,
			OBJ = 1,
			TIME_STEP = 2
		};
		enum Flag: unsigned long
		{
			DEBUG = 1<<0
		};

		Quadrotor(/*float dt,*/ int N);
		void			reset();
		void			run(float dt);
		void			step(float dt);
		bool			isset_debug() const;

		glm::vec3		angular_accel_to_torque(int, glm::vec3);
		glm::vec4		thrust_torque_to_motor_speed(
				int,
				float const &,
				glm::vec3 const &);
		void			write();
		void			write_param();
		// accessors
		glm::vec3&		x(int);
		glm::vec3&		v(int);
		glm::vec3&		a(int);
		glm::vec3&		jerk(int);
		//glm::vec3&		jounce(int);
		glm::quat&		q(int);
		glm::vec3&		omega(int);
		glm::vec3&		alpha(int);
		float			t(int i)
		{
			//return dt_ * (float)i;
			return t_[i];
		}
	public:
		unsigned long		_M_flag;	
		int			_M_stop_cause;	
		// physical constants
		float			m_, L_, R_, Asw_, rho_, CD_, A_;
		float			Kv_, Kt_, Ktau_;
		float			k_, b_;
		float			P_max_, gamma_max_;

		glm::mat3		I_;
		glm::mat3		Iinv_;

		glm::vec3		gravity_;

		glm::mat4		A4_;
		glm::mat4		A4inv_;

		Array<float>		t_;

		//float			dt_;
		int			N_;
		// current time step
		int			_M_i;
		//float*		t_;
		
		// stop criteria
		int		ti_stop_;
		
		int		ti_f_;
		
		Telem*		telem_;
		Plant*		plant_;
		Brain*		brain_;

};


#endif





