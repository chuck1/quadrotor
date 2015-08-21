#ifndef __CONTROL_LAW_JOUNCE__
#define __CONTROL_LAW_JOUNCE__

#include <string>

#include <drone/fda.h>
#include <drone/cl/ControlLaw.h>

namespace Jounce {
	class Base:
		virtual public CL::Thrust, virtual public CL::Alpha
	{
		public:
			virtual void	step(int, float) = 0;
			//virtual bool	Check(int, glm::quat) {}
			virtual bool	check(int, glm::vec3) = 0;
			virtual void	alloc(int);
			virtual void	write(std::string, int);
		public:
			Array<glm::vec3>	jounce_;
	};
	class X:
		virtual public Jounce::Base,
		virtual public CL::X<5>
	{
		public:
			bool		check(int, glm::vec3);
			virtual void	step(int, float);
			virtual void	alloc(int);
			virtual void	write(int);
	};
	class V:
		virtual public CL::V<4>, virtual public Base
	{
		public:
			virtual void	step(int, float);
			virtual bool	check(int, glm::vec3);
			virtual void	alloc(int);
			virtual void	write(int);
		public:
			//glm::mat33		C_[4];

			//Array<glm::vec3>	e_[4];		
	};
}



#endif

