
#include <quadrotor/brain.h>
#include <quadrotor/command.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/command/Stop.hpp>
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

void	Command::Stop::XCross::Check(int i) {
	
	double d = p_.GetDistance(cmd_->r_->x(i));
	
	if((d_ * d) < 0) {
		stats_.i_ = i;
		stats_.t_ = cmd_->r_->t(i);
		
		cmd_->flag_ |= Command::Base::Flag::e::COMPLETE;
	}
	
	d_ = d;
}

void	Command::Stop::VCross::Check(int i) {
	
	double d = p_.GetDistance(cmd_->r_->v(i));
	
	//printf("d=%lf\n",d);
	
	if((d_ * d) < 0) {
		stats_.i_ = i;
		stats_.t_ = cmd_->r_->t(i);

		cmd_->flag_ |= Command::Base::Flag::e::COMPLETE;	
	}
	d_ = d;
}




