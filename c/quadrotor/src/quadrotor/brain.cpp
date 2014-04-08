#include <deque>

#include <quadrotor/attitude.h>
#include <quadrotor/command.h>
#include <quadrotor/position.h>

Brain::Brain(Quadrotor* quad):
	mode_(Brain::Mode::e::IMPULSE),
	quad_(quad)
{
	pos_ = new Position(quad);
	att_ = new Attitude(quad);

	heading_ = 0.0;//M_PI * 0.5;

	int n = quad_->N_;

	f_R_.alloc(n);

	thrust_.alloc(n);

	obj_ = NULL;
}
void Brain::reset() {
	objs_.clear();
	obj_ = NULL;

	pos_->reset();
	att_->reset();
}
void Brain::step_accel(int ti) {
	
	f_R_[ti] = (pos_->a_[ti] - quad_->gravity_ /*-f_D*/) * quad_->m_;
	
	math::quat qn;

	math::vec3 z(0,0,1);
	math::quat q = quad_->telem_->q_[ti];
	
	// ignore all but z-component
	//f_R.x = 0;
	//f_R.y = 0;

	if(!q.isSane()) printf("insane\n");
	//printf("q"); q.print();
	if(f_R_[ti].isNan()) {
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
void Brain::step_impulse(double dt, int ti) {
	
	math::vec3 tmp = quad_->telem_->q_[ti].rotate(pos_->i_[ti]);
	
	double i_RB = tmp.z;
	
	thrust_[ti] = thrust_[ti-1] + i_RB * dt * quad_->m_;
	//a_RB_[ti] = a_RB_[ti-1] + i_RB * dt;

	math::vec3 o;
	if(thrust_[ti] > 0) {
		o.x = tmp.y / thrust_[ti];
		o.y = -tmp.x / thrust_[ti];
	}


	if(!o.isSane()) {
		printf("tmp\n");
		tmp.print();
		throw;
	}

	att_->set_o_reference(ti, o);


}
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
		case Brain::Mode::e::IMPULSE:
			pos_->step(dt, ti, ti_0);

			pos_->step_impulse(dt, ti, ti_0);

			step_impulse(dt, ti);

			att_->step(dt, ti, ti_0);

			att_->step_torque_rotor_body(ti, ti_0);

			break;
		default:
			throw InvalidOp(ti);
			break;
	}


	step_motor_speed(ti);
}
void Brain::step_motor_speed(int ti) {

	math::vec4 temp(
			att_->tau_RB_[ti].x,
			att_->tau_RB_[ti].y,
			att_->tau_RB_[ti].z,
			0.0);

	quad_->plant_->gamma1_[ti] = quad_->A4inv_ * temp;

	// thrust
	quad_->plant_->gamma0_[ti] = thrust_[ti] / (quad_->k_ * 4.0);

	if(!quad_->plant_->gamma1_[ti].isSane()) {
		printf("gamma1\n");
		quad_->plant_->gamma1_[ti].print();
		printf("A4\n");
		quad_->A4inv_.print();
		printf("temp\n");
		temp.print();
		throw;
	}


	//printf("gamma\n");
	//quad_->plant_->gamma_[ti].print();

}
void Brain::control_law_3(double dt, int ti, int ti_0) {
	// require position error
	pos_->step(dt, ti, ti_0);

	/*math::vec3 f_R =*/ //pos_->get_force_rotor(ti, ti_0);


	//process_force_reference(ti);

	// get body torque
	//att_->get_tau_RB(ti, ti_0);

	step_motor_speed(ti);

}	
void Brain::step(double dt, int ti) {

	if ((obj_ == NULL) || (obj_->flag_ & Command::Base::Flag::COMPLETE)) {

		if(objs_.empty()) {
			throw EmptyQueue(ti);
		}

		//print 'new move'
		obj_ = objs_.front();
		objs_.pop_front();
		ti_0_ = 0;

		switch(obj_->type_) {
			case Command::Base::Type::MOVE:
				pos_->set_obj(ti, (Command::Position*)obj_);
				break;
			case Command::Base::Type::PATH:
				pos_->set_obj(ti, (Command::Position*)obj_);
				break;
			case Command::Base::Type::ORIENT:
				// set reference altitude to current altitude
				Command::Move* move = new Command::Move(quad_->telem_->x_[ti]);
				pos_->set_obj(ti, move);

				att_->set_obj(ti, (Command::Orient*)obj_);


				break;
		}
	}

	switch(obj_->type_) {
		case Command::Base::Type::MOVE:
			control_law_position(dt, ti, ti_0_);
			break;
		case Command::Base::Type::PATH:
			control_law_position(dt, ti, ti_0_);
			break;
		case Command::Base::Type::ORIENT:
			control_law_3(dt, ti, ti_0_);
			break;
	}	

	ti_0_++;
}
void Brain::write(int ti) {
	const char * name = "brain.txt";

	FILE* file = fopen(name,"w");

	ti = (ti > 0) ? (ti) : (quad_->N_);

	fwrite(thrust_.v_,			sizeof(double),	ti, file);


	fclose(file);

	//for(int ti = 0; ti < quad_->N_; ti++) {
	// fwrite(&e1_[ti].x, sizeof(float), 1, file);
	//}

}



