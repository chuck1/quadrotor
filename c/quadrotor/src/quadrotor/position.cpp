

#include <math/mat33.h>

#include <quadrotor/brain.h>
#include <quadrotor/command.h>
#include <quadrotor/fda.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/telem.h>
#include <quadrotor/plant.h>
#include <quadrotor/position.h>

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

	C1_ = math::mat33(
			C1,0,0,
			0,C1,0,
			0,0,C1);
	C2_ = math::mat33(
			C2,0,0,
			0,C2,0,
			0,0,C2);
	C3_ = math::mat33(
			C3,0,0,
			0,C3,0,
			0,0,C3);
	C4_ = math::mat33(
			C4,0,0,
			0,C4,0,
			0,0,C4);
	C5_ = math::mat33(
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
double coeff(double* r, int n, int i, int k) {
	
	int i__ = i;
	
	double c = 0;
	
	printf("%i\n",i);

	for(; i <= k; i++) {
		if((i+2) < n) {
			printf("descend\n");
			c += r[i] * coeff(r, n, i+1, k+1);
		} else {
			printf("stop\n");
			c += r[i];
			//printf("%e %e %i\n",c,r[i],i);
		}
	}
	
	if(i__==0) printf("%e\n",-c);
	
	return -c;
}

void Position::set_poles(double* p) {

	

	double C2 = coeff(p, 5, 0, 1);

	throw;

	double C1 = coeff(p, 5, 0, 0);

	

	double C3 = coeff(p, 5, 0, 2);
	double C4 = coeff(p, 5, 0, 3);
	double C5 = coeff(p, 5, 0, 4);

	printf("poles % e % e % e % e % e\n",p[0],p[1],p[2],p[3],p[4]);
	printf("coeff % e % e % e % e % e\n",C1,C2,C3,C4,C5);

	throw;

/*
	double C1 = p[0]*p[1]*p[2]*p[3];
	double C2 = -p[0]*p[1]*p[2] - p[0]*p[1]*p[3] - p[0]*p[2]*p[3] - p[1]*p[2]*p[3];
	double C3 = p[0]*p[1] + p[0]*p[2] + p[0]*p[3] + p[1]*p[2] + p[1]*p[3] + p[2]*p[3];
	double C4 = -p[0] - p[1] - p[2] - p[3];
	double C5 = 1.0;
*/	


	C1_ = math::mat33(
			C1,0,0,
			0,C1,0,
			0,0,C1);
	C2_ = math::mat33(
			C2,0,0,
			0,C2,0,
			0,0,C2);
	C3_ = math::mat33(
			C3,0,0,
			0,C3,0,
			0,0,C3);
	C4_ = math::mat33(
			C4,0,0,
			0,C4,0,
			0,0,C4);
	C5_ = math::mat33(
			C5,0,0,
			0,C5,0,
			0,0,C5);

}
void Position::reset() {
	flag_ = 0;
}
void Position::fill_xref(int ti1, math::vec3 x) {
	for (int ti = ti1; ti < quad_->N_; ti++) x_ref_[ti] = x;
}
void Position::fill_xref_parametric(int ti1, math::vec3 (*f)(double)) {
	for(int ti = ti1; ti < quad_->N_; ti++) {
		double t = quad_->t_[ti];
		x_ref_[ti] = f(t);
	}
}


void Position::step(double dt, int ti, int ti_0) {
	//double dt = quad_->t_[ti] - quad_->t_[ti-1];

	e1_[ti] = x_ref_[ti] - quad_->telem_->x_[ti];
	e1_mag_[ti] = e1_[ti].magnitude();

	e2_[ti] = x_ref_d_[ti] - quad_->telem_->v_[ti];

	e3_[ti] = x_ref_dd_[ti] - quad_->plant_->a_[ti];

	e4_[ti] = x_ref_ddd_[ti] - quad_->telem_->jerk_[ti];

	if (ti_0 > 0) {
		chi_[ti] = chi_[ti-1] + e1_[ti] * dt;
	}

	forward(x_ref_,		x_ref_d_, dt, ti, ti_0, 0);
	forward(x_ref_d_,	x_ref_dd_, dt, ti, ti_0, 1);
	forward(x_ref_dd_,	x_ref_ddd_, dt, ti, ti_0, 2);
	forward(x_ref_ddd_,	x_ref_dddd_, dt, ti, ti_0, 3);

	forward(e1_mag_,   e1_mag_d_,  dt, ti, ti_0, 0);
	forward(e1_mag_d_, e1_mag_dd_, dt, ti, ti_0, 1);

	if(!x_ref_d_[ti].isSane()) {
		printf("x_ref_d_ is insane\n");
		printf("%i %i\n",ti,ti_0);
		x_ref_[ti-1].print();
		x_ref_[ti].print();
		throw;
	}

	// step position error



	//x_ref_[ti].print();
	//printf("e1_mag %f\n",e1_mag);

	check_command(ti);
}
void Position::check_command(int ti) {

	math::vec3 tol(0.01,0.01,0.01);

	if (pos_) {
		//printf("pos\n");

		if (pos_->mode_ == Command::Position::Mode::NORMAL) {

			bool close = e1_[ti].abs_less(pos_->thresh_);

			if (close) {

				if (e2_[ti].abs_less(tol)) {

					if (e3_[ti].abs_less(tol)) {

						if (e4_[ti].abs_less(tol)) {

							if(jounce_[ti-1].abs_less(tol)) {
								((Command::Move*)pos_)->settle(ti, quad_->t_[ti]);
							}
						}
					}
				}
			}
		} else {
			//printf("mode %i\n",pos_->mode_);
		}
	}
}
void Position::set_obj(int ti, Command::Position* pos) {
	pos_ = pos;

	// reset
	flag_ &= ~Command::Position::Flag::COMPLETE;

	Command::Move* move;
	Command::Path* path;

	if (pos == NULL) throw;

	switch(pos_->type_) {
		case Command::Base::Type::MOVE:
			move = (Command::Move*)pos_;
			fill_xref(ti, move->x2_);
			break;
		case Command::Base::Type::PATH:
			path = (Command::Path*)pos_;

			printf("ti %i path %p\n",ti,path);

			fill_xref_parametric(ti, path->f_);
			break;
	}

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
void Position::step_jounce(double, int ti, int ti_0) {

	jounce_[ti] = 
		C1_ * chi_[ti] + 
		C2_ * e1_[ti] + 
		C3_ * e2_[ti] + 
		C4_ * e3_[ti] +
		C5_ * e4_[ti] +
		x_ref_dddd_[ti];

}
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
void Position::write_param() {
	const char * name = "param/pos_param.txt";
	FILE* file = fopen(name,"w");
	if(file != NULL) {
		C1_.write(file);
		C2_.write(file);
		C3_.write(file);
		C4_.write(file);
		C5_.write(file);

		printf("write file %s\n",name);

		fclose(file);
	}
}
void Position::read_param() {
	const char * name = "param/pos_param.txt";
	FILE* file = fopen(name,"r");
	if(file != NULL) {
		C1_.read(file);
		C2_.read(file);
		C3_.read(file);
		C4_.read(file);
		C5_.read(file);

		printf("read file %s\n",name);

		fclose(file);
	} else {
		printf("no file %s\n", name);
	}
}






