#ifndef DRONE_PRINT_HPP
#define DRONE_PRINT_HPP

namespace drone {
	int	print(glm::quat const & a);
	int	print(glm::vec3 const & a);
	int	print(glm::vec4 const & a);
	int	print(glm::mat4 const & a);
	int	print(double const & a);
}

#endif


