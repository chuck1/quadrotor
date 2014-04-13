#include <InputFunc.hpp>

math::vec3 constant(double) {
	return math::vec3(1,0,0);
}

math::vec3 sinewave(double t) {
	
	double per = 15.0;
	
	return math::vec3(sin(t * 2.0 * M_PI / per), t, 0.0);
}

