#ifndef __COMMAND__
#define __COMMAND__

#include <cstdio>

#include <exception>
#include <vector>

#include <math/quat.h>
#include <math/vec3.h>

class Quadrotor;

namespace Input {
	namespace Vec3 {
		class Base;
	}
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
					FREEZE,
				};
			};
		public:
			Base(Quadrotor*, Type::e);
			virtual ~Base() {}

			virtual void	Check(int);
			virtual void	Start(int) {}
		public:
			Quadrotor*	r_;

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;

		
			std::vector<Command::Stop::Base*>	stop_;
	};

	class V: public Base {
		public:
			V(Quadrotor*,  Input::Vec3::Base*);

		public:
			Input::Vec3::Base*	in_;

	};
	class X: public Base {
		public:
			X(Quadrotor*,  Input::Vec3::Base*);

		public:
			Input::Vec3::Base*	in_;
	};


	class Q: public Base {
		public:
			Q(Quadrotor*, Input::Quat*);

			Input::Quat*	in_;
	};	
	
	class Freeze: public Base {
		public:
			Freeze(Quadrotor* r): Command::Base(r, Command::Base::Type::e::FREEZE) {}
			
			virtual void	Start(int);
	};


}



#endif
