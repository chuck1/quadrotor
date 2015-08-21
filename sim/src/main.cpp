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

#include <Map.hpp>
#include <Search.hpp>
#include <InputFunc.hpp>
#include <Print.hpp>

namespace po = boost::program_options;

po::variables_map vm;

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

	auto v = std::dynamic_pointer_cast<Jounce::V>(r->brain_->cl_v_);
	auto q = std::dynamic_pointer_cast<Alpha1::Q>(r->brain_->cl_q_);
	
	//float poles_v[] = {-19.0, -19.0, -19.0, -00.0};
	float poles_v[] = {-02.1, -02.1, -01.1, 0};
	
	int i_v[] = {0,1,2,3};
	
	v->set_poles(i_v, poles_v, 4);
	
	float poles_q[] = {
		-10.0,
		-05.0,
		-00.0};
	
	int i_q[] = {0,1,2,3};
	q->set_poles(i_q, poles_q, 4);

}
void CommandScheme1(Quadrotor* r)
{
	printf("%s\n", __PRETTY_FUNCTION__);
	
	std::shared_ptr<Command::X> cmd_x(
			new Command::X(r, new Input::Vec3::Const(glm::vec3(1,0,0))));

	auto stop_x = new Command::Stop::VSettle(
			cmd_x,
			glm::vec3(0.01,0.01,0.01));

	cmd_x->stop_.push_back(stop_x);
	r->brain_->objs_.push_back(cmd_x);
}
void CommandScheme2(Quadrotor* r)
{
	std::shared_ptr<Command::V> cmd_v(
			new Command::V(r, new Input::Vec3::Const(glm::vec3(10,0,0))));

	auto stop_v = new Command::Stop::VSettle(cmd_v, glm::vec3(0.01,0.01,0.01));
	cmd_v->stop_.push_back(stop_v);
	
	std::shared_ptr<Command::Q> cmd_q(
			new Command::Q(
				r,
				new Input::QuatConst(glm::quat(0.5 * M_PI,glm::vec3(1,0,0)))));

	auto stop_q = new Command::Stop::VCross(
			cmd_q,
			math::Plane(glm::vec3(0,0,1), 0));

	cmd_q->stop_.push_back(stop_q);
	
	std::shared_ptr<Command::Freeze> cmd_freeze(new Command::Freeze(r));

	//auto cmd_freeze = new Command::V(r, new Input::Vec3::Const(glm::vec3(0,0,0)));

	r->brain_->objs_.push_back(cmd_v);
	r->brain_->objs_.push_back(cmd_q);
	r->brain_->objs_.push_back(cmd_freeze);
}
void CommandScheme3(Quadrotor* r)
{
	std::shared_ptr<Command::X> cmd_x(
			new Command::X(r, new Input::Vec3::Circle(1.0, 4.0)));
	
	//auto stop_x = new Command::Stop::VSettle(cmd_x, glm::vec3(0.01,0.01,0.01));
	//cmd_x->stop_.push_back(stop_x);
	
	r->brain_->objs_.push_back(cmd_x);
}
void CommandScheme4(Quadrotor* r)
{
	std::shared_ptr<Command::X> cmd_x(
			new Command::X(r, new Input::Vec3::Square(1.0, 4.0)));
	
	//auto stop_x = new Command::Stop::VSettle(cmd_x, glm::vec3(0.01,0.01,0.01));
	//cmd_x->stop_.push_back(stop_x);
	
	r->brain_->objs_.push_back(cmd_x);
}
float		metric_1(Quadrotor * drone)
{
	auto x = std::dynamic_pointer_cast<Jounce::X>(drone->brain_->cl_x_);
	assert(x);

	return glm::length(x->e_[0][drone->ti_stop_ - 10]);
}
void			normal(int N, float dt)
{
	std::shared_ptr<Quadrotor> drone(new Quadrotor(N));
	drone->init();

	auto x = std::dynamic_pointer_cast<Jounce::X>(drone->brain_->cl_x_);

	//float poles[] = {-6.0, -4.0, -0.0};
	//float poles[] = {-3.0, -1.1, -0.0};
	//float poles[] = {-22.0,  -9.0,   0};
	float poles[] = {-14.0,  -6.0,   0};
	int i[] = {0,0,1,1,2};
	x->set_poles(i, poles, 3);

	VQPoles(drone.get());

	CommandScheme2(drone.get());
	
	//r->brain_->objs_.push_back(new Command::X(r, sinewave));

	drone->run(dt);

	drone->write();
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
		("debug", "debug")
		;

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

	int N = 10000;

	std::shared_ptr<Quadrotor> drone(new Quadrotor(N));
	drone->init();

	if(vm.count("debug")) {
		drone->_M_flag |= Quadrotor::Flag::DEBUG;
		printf("debug set %lu\n", drone->_M_flag);
	}


	if(strcmp(mode.c_str(),"normal")==0) {
		normal(10000, dt);
	} else if(strcmp(mode.c_str(),"map")==0) {
		Map m;
	
		m._M_command_scheme_function = CommandScheme1;
		m._M_metric = metric_1;

		m.run(drone);
	} else if(strcmp(mode.c_str(),"search")==0) {
		srch();
	} else {
		printf("invalid mode \"%s\"\n", mode.c_str());
	}

	//b->att_->write();
}



