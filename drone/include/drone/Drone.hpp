#ifndef __QUADROTOR__
#define __QUADROTOR__

#include <memory>

#include <glm/glm.hpp>

#include <drone/util/decl.hpp>
#include <drone/array.h>

class Telem;
class Plant;
class Brain;

void product(int choices, int repeat, int*& arr, int level = 0);


class Drone:
	public std::enable_shared_from_this<Drone>
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

		Drone(/*float dt,*/ int N);
		void			reset();
		void			init();
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

		float			get_score(
				std::shared_ptr<Command::Base> cmd,
				std::function<float(Drone*)> metric,
				float score,
				int & N);

		std::shared_ptr<CL::Base>		get_cl();
		std::shared_ptr<Command::Base>		get_command();
	public:
		unsigned long		_M_flag;	
		int			_M_stop_cause;	
		// hardware
		std::shared_ptr<drone::hardware::MotorProp>	_M_motorprop;
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
		
		// key components
		Telem*		telem_;
		//Plant*		plant_;
		Brain*		brain_;

		// motor speeds
		Array<float>		gamma0_;
		Array<glm::vec4>	gamma1_;
};


#endif





