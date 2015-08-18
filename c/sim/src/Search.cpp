
#include <string.h>

#include <drone/Drone.hpp>
#include <drone/cl/Snap.hpp>
#include <drone/Brain.hpp>
#include <drone/command/command.h>
#include <drone/command/Stop.hpp>
#include <drone/command/Input.hpp>

#include <Search.hpp>

#include <InputFunc.hpp>
#include <Print.hpp>

search::search(int* i, float* v, int nv):
	v_(new float[nv]),
	nv_(nv),
	ts_(1e10),
	current_(0),
	n_(100000),
	count_(0)
{
	i_ = new int[5];
	memcpy(i_, i, 5*sizeof(int));

	r_ = new Quadrotor(0.01, n_);
	
	memcpy(v_, v, nv_*sizeof(float));
}
void search::step() {
	
	float low  = v_[current_] / 1.1;
	float val  = v_[current_];
	float high = v_[current_] * 1.1;
	
	v_[current_] = low;
	if(test()) {
		count_ = 0;
		printf("%i %lf %lf\n", current_, v_[current_], ts_);
		//print_arr(C_,5);
		return;
	}

	v_[current_] = high;
	if(test()) {
		count_ = 0;
		printf("%i %lf %lf\n", current_, v_[current_], ts_);
		//print_arr(C_,5);
		return;
	}
	
	count_++;
	
	printf("no change\n");
	
	v_[current_] = val;
	//print_arr(C_,5);
	current_ = (current_ + 1) % nv_;
}
void search::exec(int m) {

	if(test()) {
		for(int i = 0; i < m; i++) {
			step();

			if(count_ == nv_) {
				break;
			}
		}
		//print_arr(v_,nv_);

		n_ += 100;
		test();
		r_->write();
	} else {
		printf("failed\n");
	}
}
void search::reset()
{
	Jounce::X* x = dynamic_cast<Jounce::X*>(r_->brain_->cl_x_);	
	
	x->set_poles(i_, v_, nv_);
}
bool search::test() {
	r_->reset();

	reset();
	
	auto comm_x = new Command::X(r_, new Input::Vec3::Const(glm::vec3(1,0,0)));

	auto stop_x = new Command::Stop::XSettle(comm_x, glm::vec3(0.01,0.01,0.01));
	
	comm_x->stop_.push_back(stop_x);
	
	r_->brain_->objs_.push_back(comm_x);

	r_->ti_stop_ = n_;
	r_->run();

	Command::X* move = (Command::X*)(r_->brain_->obj_);

	if(move->flag_ & Command::Base::Flag::COMPLETE) {
		if(move->stop_.empty()) {
			printf("%s:%i\n", __FILE__, __LINE__);
			throw 0;
		}
		auto stop = move->stop_[0];
		if(stop->stats_.t_ < ts_) {
			ts_ = stop->stats_.t_;
			n_ = stop->stats_.i_;
			return true;
		}
	}
	return false;
}







