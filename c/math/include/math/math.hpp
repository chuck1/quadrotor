#ifndef MATH_MATH_HPP
#define MATH_MATH_HPP

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace math {
	bool	is_nan_or_inf(float const & v);
	bool	is_nan_or_inf(glm::vec3 const & v);
	bool	is_nan_or_inf(glm::vec4 const & v);
	bool	is_nan_or_inf(glm::quat const & v);
}


#endif
