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
					POINT,
					PATH,
					ORIENT,
					VELOCITY
				};
			};

			Base(Type::e, Mode::e, Quadrotor*);
		public:
			Quadrotor*	r_;

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;
	};

	class Position: public Base {
		public:
			Position(Base::Type::e type, Base::Mode::e, Quadrotor*);
			Position(Base::Type::e type, Base::Mode::e, math::vec3 thresh_, Quadrotor*);
			
			virtual void	check(int) = 0;

			math::vec3	thresh_;
	};


	class Point: public Command::Position {
		public:
			math::vec3	x2_;


			Point(math::vec3 x2, math::vec3 thresh, Quadrotor*);
			Point(math::vec3 x2, Quadrotor*);

			virtual void	check(int);

			void settle(int, double t);
		public:
			int		ti_s_;
			double		ts_;
	};

	class Path: public Position {
		public:
			Path(math::vec3 (*f)(double), Quadrotor*);

			virtual void	check(int);

		public:
			math::vec3 (*f_)(double);
	};
	
	
	class Orient: public Base {
		public:
			math::quat	q_;
			double		thresh_;

			Orient(math::quat q, double thresh, Quadrotor*);
			Orient(math::quat q, Quadrotor*);
	};	

}



#endif
