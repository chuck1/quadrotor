#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <memory>

#define _DEBUG 0

#include <glm/glm.hpp>

#include <boost/program_options.hpp>

//#include <drone/attitude.h>
//#include <drone/position.h>
#include <drone/Brain.hpp>
#include <drone/command/Input.hpp>
#include <drone/command/Stop.hpp>
#include <drone/Drone.hpp>
#include <drone/cl/ControlLaw.h>
#include <drone/cl/Snap.hpp>
#include <drone/cl/Alpha.hpp>

#include <Search.hpp>
#include <InputFunc.hpp>
#include <Print.hpp>

namespace po = boost::program_options;

int* range(int s, int e) {
	int* r = new int[e-s];
	for(int i = 0; i < (e-s); ++i) {
		r[i] = i + s;
	}
	return r;
}
/**
 * set control coefficients for v and q conrollers
 */
void VQPoles(Quadrotor* r)
{

	Jounce::V* v = dynamic_cast<Jounce::V*>(r->brain_->cl_v_);
	Alpha1::Q* q = dynamic_cast<Alpha1::Q*>(r->brain_->cl_q_);
	
	float poles_v[] = {
		-19.0,
		-19.0,
		-19.0,
		-00.0};
	
	int i_v[] = {0,1,2,3};
	
	v->set_poles(i_v, poles_v, 4);
	
	float poles_q[] = {
		-02.0,
		-02.0,
		-00.0};
	
	int i_q[] = {0,1,2,3};
	q->set_poles(i_q, poles_q, 4);

}

void CommandScheme1(Quadrotor* r)
{
	auto cmd_x = new Command::X(r, new Input::Vec3::Const(glm::vec3(1,0,0)));
	auto stop_x = new Command::Stop::VSettle(cmd_x, glm::vec3(0.01,0.01,0.01));
	cmd_x->stop_.push_back(stop_x);
	r->brain_->objs_.push_back(cmd_x);
}
void CommandScheme2(Quadrotor* r)
{
	auto cmd_v = new Command::V(r, new Input::Vec3::Const(glm::vec3(0,0,10)));
	auto stop_v = new Command::Stop::VSettle(cmd_v, glm::vec3(0.01,0.01,0.01));
	cmd_v->stop_.push_back(stop_v);
	
	auto cmd_q = new Command::Q(r, new Input::QuatConst(glm::quat(0.5 * M_PI,glm::vec3(1,0,0))));
	auto stop_q = new Command::Stop::VCross(cmd_q, math::Plane(glm::vec3(0,0,1), 0));
	cmd_q->stop_.push_back(stop_q);
	
	
	auto cmd_freeze = new Command::Freeze(r);
	//auto cmd_freeze = new Command::V(r, new Input::Vec3::Const(glm::vec3(0,0,0)));

	r->brain_->objs_.push_back(cmd_v);
	r->brain_->objs_.push_back(cmd_q);
	r->brain_->objs_.push_back(cmd_freeze);
}
void CommandScheme3(Quadrotor* r)
{
	auto cmd_x = new Command::X(r, new Input::Vec3::Circle(1.0, 4.0));
	auto stop_x = new Command::Stop::VSettle(cmd_x, glm::vec3(0.01,0.01,0.01));
	cmd_x->stop_.push_back(stop_x);
	r->brain_->objs_.push_back(cmd_x);
}
void set_coeff(float* center, float* length, int choices, int repeat, float* coeff)
{
	for(int r = 0; r < repeat; r++) {
		for(int c = 0; c < choices; c++) {
			coeff[r*choices + c] = center[r] + length[r] 
				* (c * 2.0 / (choices - 1.0) - 1.0);
		}
	}
}

void			reset_quadrotor(Quadrotor* r, float* C)
{
	r->reset();

	Jounce::X* x = dynamic_cast<Jounce::X*>(r->brain_->cl_x_);
	int i[] = {0,0,1,1,2};
	x->set_poles(i, C, 3);
	
	CommandScheme1(r);

}
void sub2(Quadrotor* r, float* C, float& ts, int& N, int a, int& b)
{
	//

	//Quadrotor* r = new Quadrotor(0.01, N);

	reset_quadrotor(r, C);

	r->ti_stop_ = N;

	r->run();

	Command::X* move = dynamic_cast<Command::X*>(r->brain_->obj_);
	assert(move);
	
	//auto stop_x = dynamic_cast<Command::Stop::XSettle*>(move->stop_[0]);
	auto stop_x = move->stop_[0];
	assert(stop_x);

	if(move->flag_ & Command::Base::Flag::COMPLETE) {
		if(stop_x->stats_.t_ < ts) {
			ts = stop_x->stats_.t_;
			N = stop_x->stats_.i_;
			b = a;

			printf("b %i ts %f\n", b, ts);

			r->write();
			r->write_param();
		}
	}
}
void set_C(
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
void	print(const char * s, float* t, int n)
{
	printf("%s", s);
	for(int i = 0; i < n; ++i) {
	printf("%16f", t[i]);
	}
	printf("\n");
}
int sub1(Quadrotor* r, int* arr, float* coeff, int choices, int repeat)
{
	int len = pow(choices, repeat);

	float* C = new float[repeat];

	int N = 10000;

	//Quadrotor* r = new Quadrotor(0.01, N);

	int b = -1;
	float ts = 1e10;

	// previous winner
	set_C(coeff, arr, choices, repeat, (len-1)/2, C);
	
	print("C = ", C, repeat);
	
	sub2(r, C, ts, N, (len-1)/2, b);

	for(int a = 0; a < len; a++) {
		set_C(coeff, arr, choices, repeat, a, C);
		//printf("%i %f %f %f %f %f\n",a,C[0],C[1],C[2],C[3],C[4]);

		sub2(r, C, ts, N, a, b);

	}

	return b;
}
void map()
{
	int* arr;

	float dt = 0.01;
	int N = 100000;
	Quadrotor* r = new Quadrotor(dt, N);

	VQPoles(r);

	int choices = 5;
	int repeat = 3;
	//int len = pow(choices, repeat);

	product(choices, repeat, arr);

	//float center[] = {10.0,  3.0,  7.0,  3.0};
	//float length[] = { 9.9,  2.9,  6.9,  2.9};
	// -6.878380       -2.419344
	// -14.544241       -3.550302
	// -14.194829       -3.487132	

	float center[] = {-14.2,  -3.5, -00.0};
	float length[] = { 10.0,   3.0,   0.0};

	float* coeff = new float[choices * repeat];

	printf("map start\n");

	for(int b = 0; b < 20; b++) {
		int a = -1;

		set_coeff(center, length, choices, repeat, coeff);
		a = sub1(r, arr, coeff, choices, repeat);

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
void			normal(int N, float dt)
{
	Quadrotor* r = new Quadrotor(dt, N);

	Jounce::X* x = dynamic_cast<Jounce::X*>(r->brain_->cl_x_);
	
	/*float poles[] = {
		-3.0,
		-3.0,
		-1.1,
		-1.1,
		-0.0
	};*/
	float poles[] = {
		-22.0,
		 -9.0,
		  0
	};
	/*
	float poles[] = {
		-6.0,
		-6.0,
		-4.0,
		-4.0,
		-0.0};
	*/
	//int i[] = {0,1,2,3,4};
	int i[] = {0,0,1,1,2};
	x->set_poles(i, poles, 3);
	
	VQPoles(r);
	
	CommandScheme2(r);
	
	//r->brain_->objs_.push_back(new Command::X(r, constant, ));
	//r->brain_->objs_.push_back(new Command::Move(glm::vec3(1,1,0), glm::vec3(0.01,0.01,0.01)));
	//r->brain_->objs_.push_back(new Command::X(r, circle));
	//r->brain_->objs_.push_back(new Command::X(r, sinewave));

	r->run();

	r->write();
}
void srch()
{
	//float c[] = {-10};
	float poles[] = {-3,-1.1,-0};
	int i[] = {0,0,1,1,2};
	
	search s(i,poles,3);
	
	s.exec(200);
}
int main(int ac, char ** av)
{
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("mode", po::value<std::string>(), "mode")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(ac, av, desc), vm);
	po::notify(vm);    

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 1;
	}
	
	std::string mode;
	if (vm.count("mode")) {
		mode = vm["mode"].as<std::string>();
	} else {
		std::cout << "mode not set" << std::endl;
		return 1;
	}

	// actual program

	float dt = 0.01;

	if(strcmp(mode.c_str(),"normal")==0) {
		normal(10000, dt);
	} else if(strcmp(mode.c_str(),"map")==0) {
		map();
	} else if(strcmp(mode.c_str(),"search")==0) {
		srch();
	} else {
		printf("invalid mode \"%s\"\n", mode.c_str());
	}

	//b->att_->write();
}



