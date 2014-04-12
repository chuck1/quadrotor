#include <deque>

#include <quadrotor/array.h>
#include <quadrotor/attitude.h>
#include <quadrotor/command.h>
#include <quadrotor/position.h>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Jounce.h>
#include <quadrotor/ControlLaw/Jerk.h>
#include <quadrotor/ControlLaw/Alpha.h>

Brain::Brain(Quadrotor* quad):
	mode_(Brain::Mode::e::JOUNCE),
	quad_(quad)
{
	//pos_ = new Position(quad);
	//att_ = new Attitude(quad);

	heading_ = 0.0;//M_PI * 0.5;

	//int n = quad_->N_;

	//f_R_.alloc(n);

	//thrust_.alloc(n);
	//thrust_d_.alloc(n);

	obj_ = NULL;

	//thrust_.fill(quad_->m_ * quad_->gravity_.magnitude());

	cl_x_ = new Jounce::X(quad_);
	//cl_v_ = new Jounce::V(quad_);
	//cl_q_ = new Alpha1::Q(quad_);

}
void Brain::reset() {
	objs_.clear();
	obj_ = NULL;

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
/*
void Brain::control_law_position(double dt, int ti, int ti_0) {
	// position control
	
	switch(mode_) {
		case Brain::Mode::e::ACCEL:
			pos_->step(dt, ti, ti_0);

			pos_->step_accel(dt, ti, ti_0);

			step_accel(ti);

			att_->step(dt, ti, ti_0);

			att_->step_torque_rotor_body(ti, ti_0);

			break;
		case Brain::Mode::e::JERK:
			pos_->step(dt, ti, ti_0);

			pos_->step_jerk(dt, ti, ti_0);

			step_jerk(dt, ti);

			att_->step(dt, ti, ti_0);

			att_->step_torque_rotor_body(ti, ti_0);

			break;
		case Brain::Mode::e::JOUNCE:
			pos_->step(dt, ti, ti_0);

			switch(pos_->type_) {
				case Command::Base::Type::e::POINT:
				case Command::Base::Type::e::PATH:
					pos_->step_jounce(dt, ti, ti_0);
					break;
				case Command::Base::Type::e::VELOCITY:
					pos_->step_jounce_velocity(dt, ti, ti_0);
					break;
			}

			step_jounce(ti, dt);

			break;
		default:
			throw InvalidOp();
			break;
	}


	step_motor_speed(ti);
}
*/

void	Brain::CheckCommand(int i) {
	bool pop = false;

	if (obj_ == NULL) {
		printf("obj is NULL\n");
		pop = true;
	} else {
		if(obj_->flag_ & Command::Base::Flag::COMPLETE) {
			printf("obj is complete\n");
			pop = true;
		}
	}
		
	if(pop) {
		if(objs_.empty()) {
			throw EmptyQueue();
		}

		//print 'new move'
		obj_ = objs_.front();
		objs_.pop_front();

		switch(obj_->type_) {
			case Command::Base::Type::X:
				cl_ = cl_x_;
				break;
			case Command::Base::Type::Q:
				cl_ = cl_q_;
				break;
		}
		
		cl_->command_ = obj_;
	}
}
void Brain::step(int i, double h) {

	//printf("%s\n",__PRETTY_FUNCTION__);

	CheckCommand(i);
	
	cl_->Step(i, h);
	
}
void Brain::write(int ti) {
	cl_->write(quad_->N_);

	const char * name = "data/brain.txt";

	FILE* file = fopen(name,"w");
	
	ti = (ti > 0) ? (ti) : (quad_->N_);
	
	//fwrite(thrust_.v_,			sizeof(double),	ti, file);
	
	fclose(file);

	//for(int ti = 0; ti < quad_->N_; ti++) {
	// fwrite(&e1_[ti].x, sizeof(float), 1, file);
	//}

}



