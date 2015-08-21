#ifndef __PLANT__
#define __PLANT__

#include <glm/glm.hpp>

#include <drone/array.h>

class Quadrotor;

class Plant {
	public:
		Plant(std::shared_ptr<Quadrotor> quad);

		glm::vec3	get_tau_body(int ti);
		void		step_rotor_body(int ti);
		
		glm::vec3	get_force_rotor_body(int ti);

		glm::vec3	get_force_drag_body(int ti);
		glm::vec3	get_force_drag(int ti);

		glm::vec3	get_force(int ti);

		void		step(int ti);

		void		write(int n);

	public:
		Quadrotor*	quad_;

		// state variables

		Array<float>		gamma0_;
		Array<glm::vec4>	gamma1_;
	
		Array<float>		gamma0_act_;
		Array<glm::vec4>	gamma1_act_;
	
		Array<glm::vec3>	tau_RB_;
		
		Array<glm::vec3>	f_RB_;
		
		

};




#endif
