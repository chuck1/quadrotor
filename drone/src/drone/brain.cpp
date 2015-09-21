#include <deque>

#include <drone/array.h>
//#include <drone/attitude.h>
//#include <drone/position.h>
#include <drone/command/command.h>
#include <drone/cl/ControlLaw.h>
#include <drone/cl/Alpha.hpp>
#include <drone/cl/Jerk.hpp>
#include <drone/cl/Snap.hpp>

#include <drone/Brain.hpp>

Brain::Brain(std::shared_ptr<Drone> drone):
	mode_(Brain::Mode::e::JOUNCE),
	_M_drone(drone)
{
	//pos_ = new Position(quad);
	//att_ = new Attitude(quad);

	heading_ = 0.0;//M_PI * 0.5;

	//obj_ = NULL;

	cl_x_.reset(new Jounce::X());
	cl_v_.reset(new Jounce::V());
	cl_q_.reset(new Alpha1::Q());

	cl_x_->_M_drone = _M_drone;
	cl_v_->_M_drone = _M_drone;
	cl_q_->_M_drone = _M_drone;
	
	cl_x_->init();
	cl_v_->init();
	cl_q_->init();
}
void Brain::reset() {
	objs_.clear();
	//obj_ = NULL;

	//pos_->reset();
	//att_->reset();
}
//void Brain::step_accel(int ti) {
/*	
	f_R_[ti] = (pos_->a_[ti] - quad_->gravity_ -f_D) * quad_->m_;
	
	math::quat qn;

	math::vec3 z(0,0,1);
	math::quat q = quad_->telem_->q_[ti];
	
	// ignore all but z-component
	//f_R.x = 0;
	//f_R.y = 0;

	if(!q.isSane()) printf("insane\n");
	//printf("q"); q.print();
	if(f_R_[ti].IsNan()) {
		printf("f_R is nan");
		throw;
	}

	// transform desired rotor force from inertial to body frame
	math::vec3 f_RB = q.rotate(f_R_[ti]);
	math::vec3 z_I = q.rotate(z);
	
	// match inertial z-component
	double a = z.dot(z_I);
	
	thrust_[ti] = f_RB.dot(z_I) / a;
	
	if (f_RB.dot(z) < 0.0) f_RB = -f_RB;
	
	const double unitTol = 1e-4;
	if(f_RB.magnitude() > unitTol) {
		math::quat r(f_RB,z);
		//printf("f_RB mag %e\n", f_RB.magnitude());
		qn = r * q;
	} else {
		qn = q;
	}
	
	

	// eliminate z-axis rotation
	qn.z = 0.0;
	qn.normalize();

	// tilt limiting (caused instability!!)
	a = z.dot(qn.rotate(z));
	//double tilt = M_PI * 1.1;
	//if(acos(a) > tilt) {
	//	qn = math::quat(tilt, qn.getImaginaryPart());
	//}
	
	
	// apply heading
	math::quat h(heading_, z);
	qn = h * qn;

		
	if(!q.isSane()) {
		printf("qn insane\n");
		throw;
	}

	att_->set_q_reference(ti, qn);

	//if (r.getAngle() > (M_PI / 4.0)) {
	//	thrust = 0.0;
	//}
	

}
*/
std::shared_ptr<Command::Base>	Brain::get_obj()
{
	if(objs_.empty()) {
		//printf("i=%i empty queue\n",i);
		throw EmptyQueue();
	}
	return objs_.front();
}
std::shared_ptr<CL::Base>	Brain::get_cl()
{
	auto o = get_obj();

	switch(o->type_) {
		case Command::Base::Type::X:
			return cl_x_;
			break;
		case Command::Base::Type::V:
			return cl_v_;
			break;
		case Command::Base::Type::Q:
			return cl_q_;
			break;
		default:
			assert(0);
			break;
	}
	
	return std::shared_ptr<CL::Base>();
}
void	Brain::CheckCommand(int i)
{
	bool pop = false;

	auto o = get_obj();

	//if (obj_ == NULL) {
	//	pop = true;
	//} else {
	if(o->flag_ & Command::Base::Flag::COMPLETE) {
		//printf("i=%i obj is complete\n",i);
		pop = true;
	}
	//}

	if(pop) {
		if(objs_.empty()) {
			//printf("i=%i empty queue\n",i);
			throw EmptyQueue();
		}

		//obj_ = objs_.front();
		printf("pop command\n");

		objs_.pop_front();

		o = get_obj();

		/*
		switch(o->type_) {
			case Command::Base::Type::X:
				cl_ = cl_x_;
				break;
			case Command::Base::Type::V:
				cl_ = cl_v_;
				break;
			case Command::Base::Type::Q:
				cl_ = cl_q_;
				break;
			default:
				cl_ = NULL;
				break;
		}
		*/

		o->start(i);

		//if(cl_ != NULL) {
		auto cl = get_cl();
		//cl->command_ = o;
		cl->init();
		//}
	}
}
void		Brain::step(int i, double h)
{
	//printf("%s\n",__PRETTY_FUNCTION__);

	CheckCommand(i);

	//if(cl_ != NULL) {
	auto cl = get_cl();

	//printf("%p\n", cl.get());

	cl->step(i, h);
	//}

	get_obj()->check(i);

	//cl_->Check(i);
}
void		Brain::write(int n)
{
	cl_x_->write(n);
	cl_v_->write(n);
	cl_q_->write(n);
}



