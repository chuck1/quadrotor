#ifndef __CONTROL_LAW_ALPHA__
#define __CONTROL_LAW_ALPHA__

#include <quadrotor/ControlLaw/ControlLaw.h>

namespace Alpha1 {
	class Base: virtual public CL::Alpha {
		public:
			Base(Quadrotor*);
			void		Step(int i, double h) = 0;
			bool		Check(int, math::vec3) = 0;
			virtual void	write(int) = 0;
			virtual void	alloc(int);
	};
	class Q: virtual public CL::Q<2>, virtual public Alpha1::Base {
		public:
			Q(Quadrotor* r): CL::Base(r), CL::Q<2>(r), CL::Alpha(r), Alpha1::Base(r) {}
			void		Step(int i, double h);
			bool		Check(int, math::vec3);
			virtual void	write(int);
			virtual void	alloc(int);
	};
	class Omega: public CL::Omega<2>, virtual public Alpha1::Base {
		public:
			Omega(Quadrotor* r): CL::Base(r), CL::Alpha(r), Alpha1::Base(r), CL::Omega<2>(r) {}
			void		Step(int i, double h);
			bool		Check(int, math::vec3);
			virtual void	write(int);
			virtual void	alloc(int);
	};
}

#endif



