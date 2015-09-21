#ifndef __COMMAND__
#define __COMMAND__

#include <cstdio>

#include <exception>
#include <vector>

#include <glm/glm.hpp>

#include <drone/util/decl.hpp>

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
			Base(Drone*, Type::e);
			virtual ~Base() {}

			virtual void	check(int);
			virtual void	start(int) {}
		public:
			Drone*	r_;

			unsigned int	flag_;
			unsigned int	mode_;
			unsigned int	type_;

		
			std::vector<Command::Stop::Base*>	stop_;
	};

	class V: public Base {
		public:
			V(Drone*,  Input::Vec3::Base*);

		public:
			Input::Vec3::Base*	in_;

	};
	class X: public Base {
		public:
			X(Drone*,  Input::Vec3::Base*);
			Input::Vec3::Const*	get_input_is_const();
		public:
			Input::Vec3::Base*	in_;
	};


	class Q: public Base {
		public:
			Q(Drone*, Input::Quat*);

			Input::Quat*	in_;
	};	
	
	class Freeze: public Base {
		public:
			Freeze(Drone* r): Command::Base(r, Command::Base::Type::e::FREEZE) {}
			
			virtual void	Start(int);
	};


}



#endif
