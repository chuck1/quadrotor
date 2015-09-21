#ifndef __TELEM__
#define __TELEM__

#include <memory>
#include <stdio.h>

#include <glm/glm.hpp>

#include <drone/array.h>
#include <drone/except.h>
#include <drone/util/decl.hpp>

class Telem {
	public:

	public:
		Telem(std::shared_ptr<Drone>);

		void		step(int, float);

		void		write(int ti);
		void		read();
	public:
		Drone*	quad_;

		// state variables
		Array<glm::quat>	q_;
		Array<glm::vec3>	omega_;
		Array<glm::vec3>	alpha_;

		Array<glm::vec3>	x_;
		Array<glm::vec3>	v_;
		Array<glm::vec3>	a_;
		Array<glm::vec3>	jerk_;
		Array<glm::vec3>	s_;
};


#endif



