
#include <math/Plane.hpp>

math::Plane::Plane(glm::vec3 const & n, float const & d):
	_M_n(n),
	_M_d(d)
{
}
float		math::Plane::distance(glm::vec3 const & y)
{
	return (glm::dot(_M_n, y) - _M_d) / glm::dot(_M_n, _M_n);
}

