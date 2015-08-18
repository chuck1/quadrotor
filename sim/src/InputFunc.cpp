#include <InputFunc.hpp>

glm::vec3 constantx(double) {
	return glm::vec3(1,0,0);
}
glm::vec3 constantz(double) {
	return glm::vec3(0,0,10);
}

glm::vec3 sinewave(double t) {
	
	double per = 2.0;
	double omega = 2.0 * M_PI / per;
	return glm::vec3(sin(t * omega), t, 0.0);
}

glm::vec3 circle(double t) {
	
	double per = 2.0;
	double omega = 2.0 * M_PI / per;
	return glm::vec3(sin(t * omega), cos(t * omega), 0.0);
}


