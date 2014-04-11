#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

#include <math/mat33.h>

#include <quadrotor/array.h>

class Quadrotor;

namespace Command {
	class Base;
}

namespace CL {
	class Base {
		public:
			Base(Quadrotor* r);

			//virtual void	SetCommand(int, Command::Base*) = 0;

			virtual void	Step(int, double) = 0;
		public:
			Quadrotor*	r_;

			Command::Base*	command_;
	};
	struct Terms {
		void	set_poles(double*, int);
		void	write();
		void	read();
		void	alloc(int);

		math::mat33		c_[5];
		Array<math::vec3>	e_[5];
	};
	class X: virtual public Base, public Terms {
		public:
			X(Quadrotor* r): Base(r) {}
			void			alloc(int);
		public:
			Array<math::vec3>	x_ref_[5];
	};
	class V: virtual public Base, public Terms {
		public:
			V(Quadrotor* r): Base(r) {}
			void			alloc(int);
		public:
			Array<math::vec3>	v_ref_[5];
	};
	class Q: virtual public Base, public Terms {
		public:
			Q(Quadrotor* r): Base(r) {}
			void			alloc(int);
		public:
			Array<math::quat>	q_ref_;
			Array<math::vec3>	q_ref__[2];
	};
	class Omega: virtual public Base, public Terms {
		public:
			Omega(Quadrotor* r): Base(r) {}
			void			alloc(int);
		public:
			Array<math::vec3>	omega_ref_[2];
	};

	// outputs thrust and x- and y-components of angular acceleration
	class Thrust: virtual public CL::Base {
		public:
			Thrust(Quadrotor* r): CL::Base(r) {}

			virtual void		Step(int, double);
			void			alloc(int);
		public:
			//CL::Base*		cl_;

			Array<double>		thrust_;
	};
	class Alpha: virtual public CL::Base {
		public:
			Alpha(Quadrotor* r): CL::Base(r) {}

			virtual void		Step(int, double);
			void			alloc(int);
		public:
			//CL::Base*		cl_;

			Array<math::vec3>	alpha_;
	};
}


#endif
