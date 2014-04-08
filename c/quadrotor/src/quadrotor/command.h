#ifndef __COMMAND__
#define __COMMAND__

#include <stdio.h>

#include <exception>

#include <math/quat.h>
#include <math/vec3.h>

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
					MOVE,
					PATH,
					ORIENT
				};
			};

			Base(Type::e, Mode::e);

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;
	};

	class Position: public Base {
		public:
			Position(Base::Type::e type, Base::Mode::e);
			Position(Base::Type::e type, Base::Mode::e, math::vec3 thresh_);
			
			math::vec3	thresh_;
	};


	class Move: public Command::Position {
		public:
			math::vec3	x2_;


			Move(math::vec3 x2, math::vec3 thresh);
			Move(math::vec3 x2);
			void settle(int, double t);

			int		ti_s_;
			double		ts_;
	};

	class Path: public Position {
		public:
			Path(math::vec3 (*f)(double));

			math::vec3 (*f_)(double);
	};
	
	
	class Orient: public Base {
		public:
			math::quat	q_;
			double		thresh_;

			Orient(math::quat q, double thresh);
			Orient(math::quat q);
	};	

}



#endif
