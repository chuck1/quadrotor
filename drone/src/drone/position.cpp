

#include <glm/glm.hpp>

#include <drone/brain.h>
#if 0

#include <drone/command.h>
#include <drone/fda.h>
#include <drone/quadrotor.h>
#include <drone/telem.h>
#include <drone/plant.h>
#include <drone/position.h>

Position::Position(Quadrotor* quad):
	quad_(quad),
	flag_(0)
{

	double C1 =  0.0625;
	double C2 =  0.5;
	double C3 =  1.5;
	double C4 =  2.0;
	double C5 =  1.0;

/*	
	double C1 =  1e-4;
	double C2 =  4e-3;
	double C3 =  6e-2;
	double C4 =  4e-1;
	double C5 =  1.0;
*/

	C1_ = glm::mat3(
			C1,0,0,
			0,C1,0,
			0,0,C1);
	C2_ = glm::mat3(
			C2,0,0,
			0,C2,0,
			0,0,C2);
	C3_ = glm::mat3(
			C3,0,0,
			0,C3,0,
			0,0,C3);
	C4_ = glm::mat3(
			C4,0,0,
			0,C4,0,
			0,0,C4);
	C5_ = glm::mat3(
			C5,0,0,
			0,C5,0,
			0,0,C5);
	

	//read_param();

	int n = quad_->N_;
	
	chi_.alloc(n);
	
	e1_.alloc(n);
	e2_.alloc(n);
	e3_.alloc(n);
	e4_.alloc(n);

	e1_mag_.alloc(n);
	e1_mag_d_.alloc(n);
	e1_mag_dd_.alloc(n);
	
	x_ref_.alloc(n);
	x_ref_d_.alloc(n);
	x_ref_dd_.alloc(n);
	x_ref_ddd_.alloc(n);
	x_ref_dddd_.alloc(n);

	a_.alloc(n);

	jerk_.alloc(n);
	jounce_.alloc(n);

}

void Position::reset() {
	flag_ = 0;
}




void Position::step_accel(double, int ti, int ti_0) {

	a_[ti] = 
		C1_ * chi_[ti] + 
		C2_ * e1_[ti] + 
		C3_ * e2_[ti] + 
		x_ref_dd_[ti];

}
void Position::step_jerk(double, int ti, int ti_0) {

	jerk_[ti] = 
		C1_ * chi_[ti] + 
		C2_ * e1_[ti] + 
		C3_ * e2_[ti] + 
		C4_ * e3_[ti] +
		x_ref_ddd_[ti];


}
/*
void Position::step_jounce_velocity(int i) {

	jounce_[ti] = 
		C3_ * e2_[ti] + 
		C4_ * e3_[ti] +
		C5_ * e4_[ti] +
		x_ref_dddd_[ti];
}
*/
void Position::write(int ti) {
	FILE* file = fopen("data/pos.txt","w");

	ti = (ti > 0) ? (ti) : (quad_->N_);

	e1_.write(file, ti);
	e2_.write(file, ti);
	e3_.write(file, ti);
	e4_.write(file, ti);

	x_ref_.write(file, ti);
	x_ref_d_.write(file, ti);
	x_ref_dd_.write(file, ti);
	x_ref_ddd_.write(file, ti);
	x_ref_dddd_.write(file, ti);

	a_.write(file, ti);
	jerk_.write(file, ti);
	jounce_.write(file, ti);

	e1_mag_d_.write(file, ti);
	e1_mag_dd_.write(file, ti);



	fclose(file);

	//for(int ti = 0; ti < quad_->N_; ti++) {
	// fwrite(&e1_[ti].x, sizeof(float), 1, file);
	//}

}




#endif


