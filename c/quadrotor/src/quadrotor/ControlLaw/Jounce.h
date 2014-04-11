#ifndef __CONTROL_LAW_JOUNCE__
#define __CONTROL_LAW_JOUNCE__

#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

namespace Jounce {
	class Base: virtual public CL::Thrust, virtual public CL::Alpha {
		public:
			Base(Quadrotor*);

			virtual void	Step(int, double) = 0;

		public:
			Array<math::vec3>	jounce_;
	};
	class X: virtual public CL::X, virtual public Base {
		public:
			X(Quadrotor*);
			
			void		Check(int);
			virtual void	Step(int, double);
		public:
			math::mat33		C_[5];

			Array<math::vec3>	e_[5];		
	};
	class V: virtual public CL::V, virtual public Base {
		public:
			V(Quadrotor* r): CL::Base(r), CL::V(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {}

			virtual void	Step(int, double);
		public:
			math::mat33		C_[4];

			Array<math::vec3>	e_[4];		
	};
}



#endif

