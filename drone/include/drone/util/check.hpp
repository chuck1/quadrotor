#ifndef DRONE_UTIL_CHECK_HPP
#define DRONE_UTIL_CHECK_HPP

#define DRONE_CHECK3(a,b,c)   drone::util::check(__FILE__, __LINE__, a,b,c)
#define DRONE_CHECK4(a,b,c,d) drone::util::check(__FILE__, __LINE__, a,b,c,d)

#include <stdio.h>

#include <math/math.hpp>

#include <drone/except.h>
#include <drone/util/print.hpp>

namespace drone { namespace util {

	template<typename... A>
	void	pass(A... a) {}

	template<typename A, typename... B>
	void	check(
			char const * file,
			int line,
			A a,
			B ... b)
	{
		if(math::is_nan_or_inf(a)) {
			printf("%s:%i\n",file,line);
			drone::print(a);
			pass(drone::print(b)...);
			throw Inf();
		}
	}
}}

#endif


