#include <drone/Drone.hpp>
#include <drone/Brain.hpp>
#include <drone/cl/Snap.hpp>

#include <Map.hpp>

void		print(const char * s, float* t, int n)
{
	printf("%s", s);
	for(int i = 0; i < n; ++i) {
		printf("%16f", t[i]);
	}
	printf("\n");
}

void			set_C(
		float* coeff,
		int* arr,
		int choices,
		int repeat,
		int a,
		float* C)
{
	for(int c = 0; c < repeat; c++) {
		C[c] = coeff[c*choices + arr[a*repeat + c]];
	}
}

/**
 * dim - dimensions in the map (number of coefficients)
 * 
 */
void			set_coeff(
		float* center,
		float* length,
		int choices,
		int dim,
		float* coeff)
{
	for(int d = 0; d < dim; d++) {
		for(int c = 0; c < choices; c++) {
			coeff[d * choices + c] = center[d] + length[d] 
				* (c * 2.0 / (choices - 1.0) - 1.0);
		}
	}
}


/**
 * N - maximum simulation steps
 */
void		Map::sub2(float* C, float& score, int& N, int a, int& b)
{
	reset_quadrotor(C);

	auto o = _M_drone->get_command();

	printf("_M_drone->brain_->objs_.size() = %lu\n", _M_drone->brain_->objs_.size());
	
	_M_drone->ti_stop_ = N;
	
	_M_drone->run(0.01);
	
	assert(_M_metric);
	
	float temp_score = _M_drone->get_score(o, _M_metric, score, N);

	if(temp_score < score) {
		b = a;

		score = temp_score;

		printf("b %i score %f\n", b, score);

		_M_drone->write();
		_M_drone->write_param();
	}
}
void		Map::reset_quadrotor(float* C)
{
	_M_drone->reset();
	
	auto x = std::dynamic_pointer_cast<Jounce::X>(_M_drone->brain_->cl_x_);
	assert(x);

	int i[] = {0,0,1,1,2};
	x->set_poles(i, C, 3);

	assert(_M_command_scheme_function);
	_M_command_scheme_function(_M_drone.get());

}
void		Map::run(std::shared_ptr<Quadrotor> drone)
{
	_M_drone = drone;

	int* arr;

	int N = 100000;

	//VQPoles(_M_drone);

	int choices = 5;
	int repeat = 3;
	//int len = pow(choices, repeat);

	product(choices, repeat, arr);

	//float center[] = {10.0,  3.0,  7.0,  3.0};
	//float length[] = { 9.9,  2.9,  6.9,  2.9};
	// -6.878380       -2.419344
	// -14.544241       -3.550302
	// -14.194829       -3.487132	

	float center[] = {-05.0, -02.0, -00.0};
	float length[] = { 10.0,  10.0,   0.0};

	float* coeff = new float[choices * repeat];

	printf("map start\n");

	for(int b = 0; b < 20; b++) {
		int a = -1;

		set_coeff(center, length, choices, repeat, coeff);
		a = sub1(arr, coeff, choices, repeat);

		if(a == -1) {
			printf("failed\n");
			return;
		}	

		for(int c = 0; c < repeat; c++) {
			center[c] = coeff[c*choices + arr[a*repeat + c]];
			length[c] = length[c] * 0.8;
		}

		printf("center length\n");
		//print_arr(center, repeat);
		//print_arr(length, repeat);

		//printf("a %i\n",a);
	}
}
int			Map::sub1(
		int* arr,
		float* coeff,
		int choices,
		int dim)
{
	int len = pow(choices, dim);

	float* C = new float[dim];

	int N = 10000;

	int b = -1;
	float ts = 1e10;

	// previous winner
	set_C(coeff, arr, choices, dim, (len-1)/2, C);

	print("C = ", C, dim);

	sub2(C, ts, N, (len-1)/2, b);

	for(int a = 0; a < len; a++) {
		set_C(coeff, arr, choices, dim, a, C);
		//printf("%i %f %f %f %f %f\n",a,C[0],C[1],C[2],C[3],C[4]);

		sub2(C, ts, N, a, b);
	}

	return b;
}

