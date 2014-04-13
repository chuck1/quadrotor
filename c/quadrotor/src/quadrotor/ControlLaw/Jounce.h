#ifndef __CONTROL_LAW_JOUNCE__
#define __CONTROL_LAW_JOUNCE__

#include <quadrotor/fda.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

namespace Jounce {
	class Base: virtual public CL::Thrust, virtual public CL::Alpha {
		public:
			Base(Quadrotor*);

			virtual void	Step(int, double) = 0;
			virtual void	Check(int) = 0;
			virtual void	alloc(int);
			virtual void	write(int);
		public:
			Array<math::vec3>	jounce_;
	};
	class X: virtual public CL::X<5>, virtual public Base {
		public:
			X(Quadrotor*);
			
			void		Check(int);
			virtual void	Step(int, double);
			virtual void	alloc(int);
			virtual void	write(int);
	};
	class V: virtual public CL::V<4>, virtual public Base {
		public:
			V(Quadrotor* r): CL::Base(r), CL::V<4>(r), CL::Thrust(r), CL::Alpha(r), Jounce::Base(r) {}

			virtual void	Step(int, double);
			virtual void	Check(int);
			virtual void	alloc(int);
		public:
			//math::mat33		C_[4];

			//Array<math::vec3>	e_[4];		
	};
}



#endif

