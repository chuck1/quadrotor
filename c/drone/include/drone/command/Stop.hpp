#ifndef __COMMAND_STOP__
#define __COMMAND_STOP__

#include <math/Plane.hpp>

namespace Command { namespace Stop {
		class Base {
			public:
				Base(Command::Base* cmd): cmd_(cmd) {}

				virtual void		check(int) = 0;

				struct Stats
				{
					float		t_;
					int		i_;
				};
				Stats		stats_;
				Command::Base*	cmd_;
		};
		class XSettle: public Command::Stop::Base {
			public:
				XSettle(Command::Base* cmd, glm::vec3 e):
					Command::Stop::Base(cmd), e_(e) {}

				glm::vec3	e_;
			
				virtual void		check(int);
		};
		class VSettle: public Command::Stop::Base {
			public:
				VSettle(Command::Base* cmd, glm::vec3 e):
					Command::Stop::Base(cmd), e_(e) {}

				glm::vec3	e_;
				virtual void		check(int);
		};
		class QSettle: public Command::Stop::Base {
			public:
				QSettle(Command::Base* cmd, float e):
					Command::Stop::Base(cmd), e_(e) {}

				float		e_;

				virtual void		check(int);
		};
		class XCross: public Command::Stop::Base
		{
			public:
				XCross(Command::Base* cmd, math::Plane p):
					Command::Stop::Base(cmd), p_(p), d_(0) {}

				math::Plane	p_;
				float		d_;

				virtual void		check(int);
		};
		class VCross: public Command::Stop::Base
		{
			public:
				VCross(Command::Base* cmd, math::Plane p):
					Command::Stop::Base(cmd), p_(p), d_(0) {}

				math::Plane	p_;
				float		d_;

				virtual void		check(int);
		};
		class Time: public Command::Stop::Base {
			public:
				Time(Command::Base* cmd, float t): Command::Stop::Base(cmd), t_(t) {}

				float		t_;

				virtual void		Check(int);
		};
	}
}


#endif

