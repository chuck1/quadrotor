#ifndef DRONE_SIM_MAP_HPP
#define DRONE_SIM_MAP_HPP

#include <functional>

#include <drone/util/decl.hpp>

class Map
{
public:
	void	run(std::shared_ptr<Drone>);
	int	sub1(
			int* arr, float* coeff, int choices, int repeat);
	void	sub2(
			float* C, float& score, int& N, int a, int& b);
	void	reset_quadrotor(float* C);
	
	std::shared_ptr<Drone>		_M_drone;
	std::function<float(Drone*)>	_M_metric;
	std::function<void(Drone*)>		_M_command_scheme_function;

};

#endif
