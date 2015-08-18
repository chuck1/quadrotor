#ifndef __CONTROL_LAW_ALPHA__
#define __CONTROL_LAW_ALPHA__

#include <drone/cl/ControlLaw.h>

namespace Alpha1 {
	class Base:
		virtual public CL::Alpha
	{
		public:
			Base();
			//void		init(Quadrotor*);
			void		step(int i, float h) = 0;
			bool		check(int, glm::vec3) = 0;
			virtual void	write(int) = 0;
			virtual void	alloc(int);
	};
	class Q:
		virtual public CL::Q<2>,
		virtual public Alpha1::Base
	{
		public:
			void		step(int i, float h);
			bool		check(int, glm::vec3);
			virtual void	write(int);
			virtual void	alloc(int);
	};
	class Omega: public CL::Omega<2>, virtual public Alpha1::Base {
		public:
			void		step(int i, float h);
			bool		check(int, glm::vec3);
			virtual void	write(int);
			virtual void	alloc(int);
	};
}

#endif



