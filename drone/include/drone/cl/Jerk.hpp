#ifndef __CONTROL_LAW_JERK__
#define __CONTROL_LAW_JERK__

#include <drone/fda.h>
#include <drone/Drone.hpp>
#include <drone/cl/ControlLaw.h>
#include <drone/cl/Alpha.hpp>

namespace Jerk {
	class Base:
		virtual public CL::Thrust, virtual public Alpha1::Omega
	{
		public:
			Base(Drone*);

			virtual void	step(int, float) = 0;
			virtual void	alloc(int) = 0;
			virtual void	write(int);
		public:
			Array<glm::vec3>	jerk_;
	};
	class X:
		virtual public CL::X<4>, virtual public Jerk::Base
	{
		public:
			X(Drone*);

			void		step(int, float);
			virtual bool	check(int, glm::vec3);
			virtual void	alloc(int) = 0;
			virtual void	write(int);
		public:
			//glm::mat33	C_[4];

			//Array<glm::vec3>	e_[4];
			//Array<glm::vec3>	x_ref_[4];
	};
}


#endif

