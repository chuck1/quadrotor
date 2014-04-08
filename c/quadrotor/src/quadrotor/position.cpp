

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

	double C1 = 18.97;
	double C2 = 4.90;
	
	double C1_11 = C1;
	double C1_22 = C1;
	double C1_33 = C1;

	double C2_11 = C2;
	double C2_22 = C2;
	double C2_33 = C2;

	double C3_11 = 0.0;
	double C3_22 = 0.0;
	double C3_33 = 0.0;

	double C4 = 5.0;

	C1_ = math::mat33(
			C1_11,0,0,
			0,C1_22,0,
			0,0,C1_33);
	C2_ = math::mat33(
			C2_11,0,0,
			0,C2_22,0,
			0,0,C2_33);
	C3_ = math::mat33(
			C3_11,0,0,
			0,C3_22,0,
			0,0,C3_33);
	
	C4_ = math::mat33(
			C4,0,0,
			0,C4,0,
			0,0,C4);

	//read_param();

	int n = quad_->N_;
	
	chi_.alloc(n);
	
	e1_.alloc(n);
	e2_.alloc(n);
	e3_.alloc(n);

	e1_mag_.alloc(n);
	e1_mag_d_.alloc(n);
	e1_mag_dd_.alloc(n);
	
	x_ref_.alloc(n);
	x_ref_d_.alloc(n);
	x_ref_dd_.alloc(n);
	x_ref_ddd_.alloc(n);

	a_.alloc(n);

	i_.alloc(n);

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

	if (ti_0 > 0) {
		chi_[ti] = chi_[ti-1] + e1_[ti] * dt;
	}

	forward(x_ref_,		x_ref_d_,  dt, ti, ti_0, 0);
	forward(x_ref_d_,	x_ref_dd_, dt, ti, ti_0, 1);
	forward(x_ref_dd_,	x_ref_ddd_, dt, ti, ti_0, 1);

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




	if (pos_) {
		//printf("pos\n");

		if (pos_->mode_ == Command::Position::Mode::NORMAL) {
			//printf("normal\n");

			bool close = e1_[ti].abs_less(pos_->thresh_);

			//if(close) printf("close\n");

			if (ti_0 > 1) {
				if (e1_mag_dd_[ti] > 0.0) {
					if (e1_mag_dd_[ti] < 0.01) {
						if (e1_mag_d_[ti] < 0.0) {
							if (e1_mag_d_[ti] > -0.01) {
								if (close) {

									//print 'e5 ',e5[ti]
									//print 'x ',c.x[ti]
									//print 'x_ref',x_ref[ti]
									((Command::Move*)pos_)->settle(ti, quad_->t_[ti]);
									//pos_->flag_ |= Command::Position::Flag::COMPLETE;
								}
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
		C1_ * e1_[ti] + 
		C2_ * e2_[ti] + 
		C3_ * chi_[ti] + 
		x_ref_dd_[ti];
	
}
void Position::step_impulse(double, int ti, int ti_0) {

	math::vec3 a_g = quad_->gravity_;
	
	math::vec3 f_D = quad_->plant_->get_force_drag(ti);
	
	// reference impulse
	i_[ti] = 
		C3_ * chi_[ti] + 
		C1_ * e1_[ti] + 
		C2_ * e2_[ti] + 
		C4_ * e3_[ti] +
		x_ref_ddd_[ti];
		
}

void Position::write(int ti) {
	FILE* file = fopen("pos.txt","w");

	ti = (ti > 0) ? (ti) : (quad_->N_);

	e1_.write(file, ti);
	e2_.write(file, ti);
	e3_.write(file, ti);
	
	x_ref_.write(file, ti);
	x_ref_d_.write(file, ti);
	x_ref_dd_.write(file, ti);
	a_.write(file, ti);
	i_.write(file, ti);
	
	e1_mag_d_.write(file, ti);
	e1_mag_dd_.write(file, ti);

	

	fclose(file);

	//for(int ti = 0; ti < quad_->N_; ti++) {
	// fwrite(&e1_[ti].x, sizeof(float), 1, file);
	//}

}
void Position::write_param() {
	const char * name = "pos_param.txt";
	FILE* file = fopen(name,"w");
	if(file != NULL) {
		C1_.write(file);
		C2_.write(file);
		C3_.write(file);
		C4_.write(file);

		printf("read file %s\n",name);
	}
	fclose(file);
}
void Position::read_param() {
	const char * name = "pos_param.txt";
	FILE* file = fopen(name,"r");
	if(file != NULL) {
		C1_.read(file);
		C2_.read(file);
		C3_.read(file);
		C4_.read(file);

		printf("write file %s\n",name);
	}
	fclose(file);
}






