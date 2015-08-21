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
float			math::coeff(
		float* roots,
		int n,
		int k)
{
	return math::coeff2(roots, n, 0, k);
}
float			math::coeff2(
		float* roots,
		int n,
		int i,
		int k)
{
	// i must start at 0

	float c = 0;

	//printf("%i\n",i);

	for(; i <= k; i++) {
		//printf("%i %i\n",i,k);
		if((k+1) < n) {
			//printf("descend\n");
			c += roots[i] * coeff2(roots, n, i+1, k+1);
		} else {
			//printf("stop\n");
			c += roots[i];
			//printf("%e %e %i\n",c,r[i],i);
		}
	}

	return -c;
}







