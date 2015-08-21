
#include <drone/Brain.hpp>
#include <drone/command/command.h>
#include <drone/command/Stop.hpp>
#include <drone/Drone.hpp>
#include <drone/cl/ControlLaw.h>

void	Command::Stop::XSettle::check(int i)
{
	auto cmd = cmd_.lock();
	assert(cmd);

	if(cmd->r_->get_cl()->check(i, e_)) {
		stats_.i_ = i;
		stats_.t_ = cmd->r_->t(i);

		cmd->flag_ |= Command::Base::Flag::e::COMPLETE;
	}
	
}
void	Command::Stop::VSettle::check(int i)
{
	auto cmd = cmd_.lock();
	assert(cmd);
	
	if(cmd->r_->get_cl()->check(i, e_)) {
		stats_.i_ = i;
		stats_.t_ = cmd->r_->t(i);

		cmd->flag_ |= Command::Base::Flag::e::COMPLETE;	
	}
	
}
void	Command::Stop::XCross::check(int i)
{
	auto cmd = cmd_.lock();
	assert(cmd);

	float d = p_.distance(cmd->r_->x(i));
	
	if((d_ * d) < 0) {
		stats_.i_ = i;
		stats_.t_ = cmd->r_->t(i);
		
		cmd->flag_ |= Command::Base::Flag::e::COMPLETE;
	}
	
	d_ = d;
}
void	Command::Stop::VCross::check(int i)
{
	auto cmd = cmd_.lock();
	assert(cmd);

	float d = p_.distance(cmd->r_->v(i));
	
	//printf("d=%lf\n",d);
	
	if((d_ * d) < 0) {
		stats_.i_ = i;
		stats_.t_ = cmd->r_->t(i);

		cmd->flag_ |= Command::Base::Flag::e::COMPLETE;	
	}
	d_ = d;
}




