#include <math/math.hpp>

bool	math::is_nan_or_inf(float const & v)
{
	if(isnan(v)) return true;
	if(isinf(v)) return true;
	return false;
}
bool	math::is_nan_or_inf(glm::vec3 const & v)
{
	if(glm::any(glm::isnan(v))) return true;
	if(glm::any(glm::isinf(v))) return true;
	return false;
}
bool	math::is_nan_or_inf(glm::vec4 const & v)
{
	if(glm::any(glm::isnan(v))) return true;
	if(glm::any(glm::isinf(v))) return true;
	return false;
}
bool	math::is_nan_or_inf(glm::quat const & v)
{
	if(isnan(v.x)) return true;
	if(isnan(v.y)) return true;
	if(isnan(v.z)) return true;
	if(isnan(v.w)) return true;
	if(isinf(v.x)) return true;
	if(isinf(v.y)) return true;
	if(isinf(v.z)) return true;
	if(isinf(v.w)) return true;
	return false;
}


