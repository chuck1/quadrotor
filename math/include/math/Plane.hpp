#ifndef MATH_PLANE_HPP
#define MATH_PLANE_HPP

#include <glm/glm.hpp>

namespace math {
	class Plane
	{
		public:
			Plane(glm::vec3 const &, float const &);
			float	distance(glm::vec3 const & y);
		private:
			glm::vec3	_M_n;
			float		_M_d;
	};
}

#endif

