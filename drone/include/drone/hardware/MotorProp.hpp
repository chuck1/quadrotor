#ifndef DRONE_HARDWARE_MOTORPROP_HPP
#define DRONE_HARDWARE_MOTORPROP_HPP

#include <memory>

#include <drone/util/decl.hpp>

namespace drone { namespace hardware {

	class MotorProp
	{
	public:
		float			thrust_max();
		std::shared_ptr<Motor>	_M_motor;
		std::shared_ptr<Prop>	_M_prop;
	};

}}

#endif


