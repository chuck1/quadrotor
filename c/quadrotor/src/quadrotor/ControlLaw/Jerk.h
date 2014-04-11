#ifndef __CONTROL_LAW_JERK__
#define __CONTROL_LAW_JERK__

#include <quadrotor/fda.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Alpha.h>

namespace Jerk {
	class Base: virtual public CL::Thrust, virtual public Alpha1::Omega {
		public:
			Base(Quadrotor*);

			virtual void	Step(int, double) = 0;

		public:
			Array<math::vec3>	jerk_;
	};
	class X: virtual public CL::X, virtual public Jerk::Base {
		public:
			X(Quadrotor*);

			void		step(int, double);
		public:
			math::mat33	C_[4];

			Array<math::vec3>	e_[4];
			Array<math::vec3>	x_ref_[4];
	};
}


#endif

