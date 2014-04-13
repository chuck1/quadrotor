
#include <quadrotor/brain.h>
#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

void	Command::Stop::XSettle::Check(int i) {
	if(cmd_->r_->brain_->cl_->Check(i, e_)) {
		stats_.i_ = i;
		stats_.t_ = cmd_->r_->t(i);

		cmd_->flag_ |= Command::Base::Flag::e::COMPLETE;
	}
	
}

void	Command::Stop::VSettle::Check(int i) {
	if(cmd_->r_->brain_->cl_->Check(i, e_)) {
		stats_.i_ = i;
		stats_.t_ = cmd_->r_->t(i);

		cmd_->flag_ |= Command::Base::Flag::e::COMPLETE;	
	}
	
}

