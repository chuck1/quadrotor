

#include <math/vec3.h>
#include <math/quat.h>
#include <math/mat33.h>

#include <quadrotor/attitude.h>
#include <quadrotor/command.h>
#include <quadrotor/fda.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/telem.h>
#include <quadrotor/plant.h>

Attitude::Attitude(Quadrotor* quad):
	mode_(Attitude::Mode::e::VEL),
	quad_(quad)
{

	double C1 = 15.88;
	double C2 =  5.2;
	
	C1_ = math::mat33(
		C1,0.0,0.0,
		0.0,C1,0.0,
		0.0,0.0,C1);

	C2_ = math::mat33(
		C2,0.0,0.0,
		0.0,C2,0.0,
		0.0,0.0,C2);

	//read_param();

	int n = quad_->N_;

	printf("allocate e1 %i\n", n);
	e1_.alloc(n);

	e2_.alloc(n);

	e1_mag_.alloc(n);
	e1_mag_d_.alloc(n);
	
	q_ref_.alloc(n);
	q_ref_d_.alloc(n);
	q_ref_dd_.alloc(n);

	o_ref_.alloc(n);
	o_ref_d_.alloc(n);

	tau_RB_.alloc(n);

	att_ = NULL;
}
void Attitude::reset() {
	
}
void Attitude::set_q_reference(int ti, math::quat q) {
	q_ref_[ti] = q;
}
void Attitude::set_o_reference(int ti, math::vec3 o) {
	o_ref_[ti] = o;
}
void Attitude::set_o_reference(int ti, double x, double y, double z) {
	o_ref_[ti].x() = x;
	o_ref_[ti].y() = y;
	o_ref_[ti].z() = z;

}
/*
void Attitude::set_obj(int ti1, Command::Q* att) {
	att_ = att;
	
	for (int ti = ti1; ti < quad_->N_; ti++) q_ref_[ti] = att_->q_;
}*/	
void Attitude::step(double dt, int ti, int ti_0) {
}
void Attitude::step_torque_rotor_body_att(int ti, int ti_0) {

	math::vec3 od = 
		C1_ * e1_[ti].getImaginaryPart() + 
		C2_ * e2_[ti] + 
		q_ref_dd_[ti];
	
	tau_RB_[ti] = quad_->angular_accel_to_torque(ti, od);
}
void Attitude::step_torque_rotor_body_vel(int ti, int ti_0) {
	
	//tau_RB_[ti] = quad_->angular_accel_to_torque(ti, od);
/*
	if(!tau_RB_[ti].isSane()) {
		printf("e2\n");
		e2_[ti].print();
		printf("od\n");
		od.print();
		throw;
	}*/
}
void Attitude::step_torque_rotor_body(int ti, int ti_0) {

	switch(mode_) {
		case Attitude::Mode::e::ATT:
			step_torque_rotor_body_att(ti, ti_0);
			break;
		case Attitude::Mode::e::VEL:
			step_torque_rotor_body_vel(ti, ti_0);
			break;
		default:
			throw InvalidOp();
			break;
	}
}
void Attitude::step_torque_rotor_body(int ti, math::vec3 od) {

	tau_RB_[ti] = quad_->angular_accel_to_torque(ti, od);
}
void Attitude::write(int n) {
	FILE* file = fopen("data/att.txt","w");

	n = (n > 0) ? (n) : (quad_->N_);
/*
	math::vec3* e1 = new math::vec3[n];
	math::vec3* q = new math::vec3[n];
	math::vec3* q_ref = new math::vec3[n];

	for(int ti = 0; ti < n; ti++) {
		e1[ti] = e1_[ti].getImaginaryPart();
		q[ti] = quad_->telem_->q_[ti].getImaginaryPart();
		q_ref[ti] = q_ref_[ti].getImaginaryPart();
	}
*/	
	
	e1_.write(file, n);

	q_ref_.write(file, n);
	q_ref_d_.write(file, n);
	q_ref_dd_.write(file, n);

	tau_RB_.write(file, n);

	fclose(file);
}
void Attitude::write_param() {
	const char * name = "param/att_param.txt";

	FILE* file = fopen(name,"w");

	if(file != NULL) {
		C1_.write(file);
		C2_.write(file);

		printf("write file %s\n",name);
	}
	fclose(file);
}
void Attitude::read_param() {
	const char * name = "param/att_param.txt";

	FILE* file = fopen(name,"r");

	if(file != NULL) {
		C1_.read(file);
		C2_.read(file);

		fclose(file);

		printf("read file %s\n",name);
	} else {
		printf("no file %s\n", name);
	}
	
}



