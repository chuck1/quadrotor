#ifndef DRONE_UTIL_CHECK_HPP
#define DRONE_UTIL_CHECK_HPP


#include <stdio.h>

#include <math/math.hpp>

#include <drone/except.h>
#include <drone/util/decl.hpp>
#include <drone/util/print.hpp>
#include <drone/Drone.hpp>

#define DRONE_CHECK3(drone,a,b,c) drone::util::check(drone, __FILE__, __LINE__, a,b,c)
#define DRONE_CHECK4(a,b,c,d) drone::util::check(0, __FILE__, __LINE__, a,b,c,d)

namespace drone { namespace util {

	template<typename... A>
	void	pass(A... a) {}

	template<typename A, typename... B>
	void	check(
			Drone* drone,
			char const * file,
			int line,
			A a,
			B ... b)
	{
		bool p = true;
		if(drone) {
			p = drone->isset_debug();
		}

		if(math::is_nan_or_inf(a)) {
			
			if(p) {
			printf("%s:%i\n",file,line);
			drone::print(a);
			pass(drone::print(b)...);
			}

			throw Inf();
		}
	}
}}

#endif


