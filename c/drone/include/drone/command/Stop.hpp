#ifndef __COMMAND_STOP__
#define __COMMAND_STOP__

#include <math/Plane.hpp>

namespace Command {
	namespace Stop {
		class Base {
			public:
				Base(Command::Base* cmd): cmd_(cmd) {}

				virtual void		Check(int) = 0;

				struct {
					double		t_;
					int		i_;
				} stats_;	Command::Base*	cmd_;
		};
		class XSettle: public Command::Stop::Base {
			public:
				XSettle(Command::Base* cmd, glm::vec3 e): Command::Stop::Base(cmd), e_(e) {}

				glm::vec3	e_;
			
				
				virtual void		Check(int);
		};
		class VSettle: public Command::Stop::Base {
			public:
				VSettle(Command::Base* cmd, glm::vec3 e): Command::Stop::Base(cmd), e_(e) {}

				glm::vec3	e_;
				virtual void		Check(int);
		};
		class QSettle: public Command::Stop::Base {
			public:
				QSettle(Command::Base* cmd, double e): Command::Stop::Base(cmd), e_(e) {}

				double		e_;

				virtual void		Check(int);
		};
		class XCross: public Command::Stop::Base {
			public:
				XCross(Command::Base* cmd, glm::plane p): Command::Stop::Base(cmd), p_(p), d_(0) {}

				glm::plane	p_;
				double		d_;

				virtual void		Check(int);
		};
		class VCross: public Command::Stop::Base {
			public:
				VCross(Command::Base* cmd, glm::plane p): Command::Stop::Base(cmd), p_(p), d_(0) {}

				glm::plane	p_;
				double		d_;

				virtual void		Check(int);
		};
		class Time: public Command::Stop::Base {
			public:
				Time(Command::Base* cmd, double t): Command::Stop::Base(cmd), t_(t) {}

				double		t_;

				virtual void		Check(int);
		};
	}
}


#endif

