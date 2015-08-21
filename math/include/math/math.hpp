#ifndef MATH_MATH_HPP
#define MATH_MATH_HPP

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace math {
	bool	is_nan_or_inf(float const & v);
	bool	is_nan_or_inf(glm::vec3 const & v);
	bool	is_nan_or_inf(glm::vec4 const & v);
	bool	is_nan_or_inf(glm::quat const & v);

	/** coeff
	 * return kth coefficient to a polynomial with n roots stored in r
	 * 
	 * roots - (in) roots of the polynomial
	 * n - number of roots
	 * k - power of x for which the coefficient will be returned
	 */
	float			coeff(
			float* roots,
			int n,
			int k);
	/**
	 * should not be called by user
	 * used for recusion of coeff
	 */
	float			coeff2(
			float* roots,
			int n,
			int i,
			int k);
}


#endif


