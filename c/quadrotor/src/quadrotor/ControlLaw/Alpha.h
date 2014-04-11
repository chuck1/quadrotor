#ifndef __CONTROL_LAW_ALPHA__
#define __CONTROL_LAW_ALPHA__

#include <quadrotor/ControlLaw/ControlLaw.h>

namespace Alpha1 {
	class Base: virtual public CL::Alpha {
		public:
			Base(Quadrotor* r): CL::Base(r), CL::Alpha(r) {}
	};
	class Q: virtual public CL::Q, virtual public Alpha1::Base {
		public:
			Q(Quadrotor* r): CL::Base(r), CL::Q(r), CL::Alpha(r), Alpha1::Base(r) {}
	};
	class Omega: virtual public CL::Omega, virtual public Alpha1::Base {
		public:
			Omega(Quadrotor* r): CL::Base(r), CL::Omega(r), CL::Alpha(r), Alpha1::Base(r) {}
			void	Step(int i, double h);
	};
	
}

#endif
