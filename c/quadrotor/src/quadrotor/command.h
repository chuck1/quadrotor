#ifndef __COMMAND__
#define __COMMAND__

#include <stdio.h>

#include <exception>

#include <math/quat.h>
#include <math/vec3.h>

class Quadrotor;

namespace Command {

	class Base {
		public:
			struct Flag {
				enum e {
					RISE = 1 << 0,
					SETTLE = 1 << 1,
					COMPLETE = 1 << 2,
				};
			};

			struct Mode {
				enum e {
					NORMAL,
					HOLD
				};
			};

			struct Type {
				enum e {
					X,
					V,
					Q,
				};
			};
		public:
			Base(Type::e, Mode::e, Quadrotor*);
			virtual ~Base() {}
		public:
			Quadrotor*	r_;

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;

			double		ts_;
			int		ti_s_;
	};

	class X: public Base {
		public:
			X(Quadrotor*, math::vec3 (*)(double));
			X(Quadrotor*, math::vec3 (*)(double), math::vec3 const &);

			void		Settle(int, double t);
		public:
			math::vec3	(*f_)(double);
			math::vec3	thresh_;
	};


	class Q: public Base {
		public:
			math::quat 	(*f_)(double);
			double		thresh_;

			Q(Quadrotor*, math::quat (*)(double), double thresh);
			Q(Quadrotor*, math::quat (*)(double));
	};	

}



#endif
