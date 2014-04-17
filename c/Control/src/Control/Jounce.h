#ifndef __CONTROL_LAW_JOUNCE__
#define __CONTROL_LAW_JOUNCE__

#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

namespace Jounce {
	class Base: virtual public CL::Thrust, virtual public CL::Alpha {
		public:
			Base(Quadrotor*);

			virtual void	Step(int, double) = 0;
			//virtual bool	Check(int, math::quat) {}
			virtual bool	Check(int, math::vec3) = 0;
			virtual void	alloc(int);
			virtual void	write(int);
		public:
			Array<math::vec3>	jounce_;
	};
	class X: virtual public CL::X<5>, virtual public Base {
		public:
			X(Quadrotor*);
			
			bool		Check(int, math::vec3);
			virtual void	Step(int, double);
			virtual void	alloc(int);
			virtual void	write(int);
	};
	class V: virtual public CL::V<4>, virtual public Base {
		public:
			V(Quadrotor* r);

			virtual void	Step(int, double);
			virtual bool	Check(int, math::vec3);
			virtual void	alloc(int);
			virtual void	write(int);
		public:
			//math::mat33		C_[4];

			//Array<math::vec3>	e_[4];		
	};
}



#endif

