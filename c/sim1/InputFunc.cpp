#include <InputFunc.hpp>

math::vec3 constantx(double) {
	return math::vec3(1,0,0);
}
math::vec3 constantz(double) {
	return math::vec3(0,0,10);
}

math::vec3 sinewave(double t) {
	
	double per = 2.0;
	double omega = 2.0 * M_PI / per;
	return math::vec3(sin(t * omega), t, 0.0);
}

math::vec3 circle(double t) {
	
	double per = 2.0;
	double omega = 2.0 * M_PI / per;
	return math::vec3(sin(t * omega), cos(t * omega), 0.0);
}


