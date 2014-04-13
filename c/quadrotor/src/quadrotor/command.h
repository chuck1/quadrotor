#ifndef __COMMAND__
#define __COMMAND__

#include <cstdio>

#include <exception>
#include <vector>

#include <math/quat.h>
#include <math/vec3.h>

class Quadrotor;

namespace Input {
	class Vec3;
	class Quat;
}
namespace Command {
	namespace Stop {
		class Base;
	}
	class Base {
		public:
			struct Flag {
				enum e {
					RISE = 1 << 0,
					SETTLE = 1 << 1,
					COMPLETE = 1 << 2,
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
			Base(Quadrotor*, Type::e);
			virtual ~Base() {}

			virtual void	Check(int);
		public:
			Quadrotor*	r_;

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;

		
			std::vector<Command::Stop::Base*>	stop_;
	};
	namespace Stop {
		class Base {
			public:
				Base(Command::Base* cmd): cmd_(cmd) {}

				virtual void		Check(int) = 0;

				Command::Base*	cmd_;
		};
		class XSettle: public Command::Stop::Base {
			public:
				XSettle(Command::Base* cmd, math::vec3 e): Command::Stop::Base(cmd), e_(e) {}

				math::vec3	e_;
				struct {
					double		t_;
					int		i_;
				} stats_;
				
				virtual void		Check(int);
		};
		class VSettle: public Command::Stop::Base {
			public:
				VSettle(Command::Base* cmd, math::vec3 e): Command::Stop::Base(cmd), e_(e) {}

				math::vec3	e_;
				struct {
					double		t_;
					int		i_;
				} stats_;

				virtual void		Check(int);
		};
		class QSettle: public Command::Stop::Base {
			public:
				QSettle(Command::Base* cmd, double e): Command::Stop::Base(cmd), e_(e) {}

				double		e_;

				virtual void		Check(int);
		};
		class ZCross: public Command::Stop::Base {
			public:
				ZCross(Command::Base* cmd, double z): Command::Stop::Base(cmd), z_(z), s_(0) {}

				double		z_;
				int		s_;

				virtual void		Check(int);
		};
		class Time: public Command::Stop::Base {
			public:
				Time(Command::Base* cmd, double t): Command::Stop::Base(cmd), t_(t) {}

				double		t_;

				virtual void		Check(int);
		};
	}
	class V: public Base {
		public:
			V(Quadrotor*,  Input::Vec3*);

		public:
			Input::Vec3*	in_;

	};
	class X: public Base {
		public:
			X(Quadrotor*,  Input::Vec3*);

		public:
			Input::Vec3*	in_;
	};


	class Q: public Base {
		public:
			Q(Quadrotor*, Input::Quat*);

			Input::Quat*	in_;
	};	




}



#endif
