

#include <glm/glm.hpp>

#include <stdio.h>

#include <drone/command/command.h>
#include <drone/command/Input.hpp>
#include <drone/command/Stop.hpp>
#include <drone/Drone.hpp>
#include <drone/Brain.hpp>
//#include <drone/position.h>
#include <drone/cl/ControlLaw.h>


void	Command::Freeze::Start(int i) {
	printf("%s\n",__PRETTY_FUNCTION__);
	
	glm::vec3& x = r_->x(i);
	//x.print();
	
	std::shared_ptr<Command::X> cmd_x(new Command::X(r_, new Input::Vec3::Const(x)));

	auto stop_x = new Command::Stop::XSettle(cmd_x, glm::vec3(0.01,0.01,0.01));
	cmd_x->stop_.push_back(stop_x);
	
	r_->brain_->objs_.push_back(cmd_x);
	
	flag_ |= Command::Base::Flag::e::COMPLETE;	
}









