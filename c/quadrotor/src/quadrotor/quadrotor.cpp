
#include <quadrotor/attitude.h>
#include <quadrotor/brain.h>
#include <quadrotor/telem.h>
#include <quadrotor/plant.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/position.h>


Quadrotor::Quadrotor(double dt, int N):
	dt_(dt),
	N_(N),
	ti_stop_(N),
	ti_f_(0)
{
	printf("dt %lf\n",dt);
	
	t_ = new double[N_];
	for(int ti = 0; ti < N_; ti++) t_[ti] = dt * (double)ti;

	// physical constants
	m_	= 1.0;		// mass (kg)
	L_	= 0.25;		// arm length (m)
	R_	= 0.05;		// prop radius (m)
	Asw_	= M_PI * R_ * R_;
	
	rho_ 	= 1.28;	// (kg/m3) air density
	CD_ 	= 1.0;	// dimensionless const
	A_	= 0.05 * 0.01;	// prop cross-sectional area (m2)


	Kv_	= 1450;	// back EMF (RPM / V)
	Kv_ = 1.0 / Kv_ * 0.1047;

	Kt_	= Kv_;
	Ktau_	= 0.5;

	k_ = Kv_ * Ktau_ * sqrt(rho_ * Asw_);
	b_ = 0.5 * pow(R_,3) * rho_ * CD_ * A_;
	
	//I_ = ;
	Iinv_ = I_.GetInverse();
	
	P_max_		= 340.0;
	//gamma_max_	= pow(P_max_ * sqrt(2.0 * rho_ * Asw_) / pow(k_, 3.0/2.0), 2.0/3.0);
	gamma_max_ = 1e10;
	

	printf("gamma max %e\n", gamma_max_);

	//printf("I Iinv\n");
	//I_.print();
	//Iinv_.print();

	// matrices
	A4_ = math::mat44(
			L_ * k_,	0,		-L_ * k_,	0,
			0,		L_ * k_,	0,		-L_ * k_,
			b_,		-b_,		b_,		-b_,
			1.0,		1.0,		1.0,		1.0);
	
	
	A4_.Transpose();
	
	A4inv_ = A4_.GetInverse();

	gravity_ = math::vec3(0,0,-9.81);

	telem_ = new Telem(this);
	plant_ = new Plant(this);
	brain_ = new Brain(this);
}
math::vec3&	Quadrotor::x(int i) { return telem_->x_[i]; }
math::vec3&	Quadrotor::v(int i) { return telem_->v_[i]; }
math::vec3&	Quadrotor::a(int i) { return telem_->a_[i]; }
math::vec3&	Quadrotor::jerk(int i) { return telem_->jerk_[i]; }
//math::vec3&	Quadrotor::jounce(int i) { return telem_->jounce_[i]; }
math::quat&	Quadrotor::q(int i) { return telem_->q_[i]; }
math::vec3&	Quadrotor::omega(int i) { return telem_->omega_[i]; }
math::vec3&	Quadrotor::alpha(int i) { return telem_->alpha_[i]; }
void Quadrotor::reset() {
	ti_f_ = 0;

	brain_->reset();
}
void Quadrotor::run() {

	//printf("dt %f\n", dt_);

	printf("command queue %i\n", (int)brain_->objs_.size());

	for(int ti = 1; ti < ti_stop_; ti++) {

		if ((ti % (N_ / 100)) == 0) {
			//printf("%i %f\n",ti,t[ti]);
		}

		try {
			brain_->step(ti-1, dt_);
			plant_->step(ti-1);
			telem_->step(ti, dt_);
		}
		catch(EmptyQueue &e) {
			printf("empty queue ti\n");
			ti_f_ = ti;
			break;
		}
		catch(StopCond &e) {
			//printf("%s\n", e.what());
			ti_f_ = ti;
			break;
		}
		catch(...) {
			ti_f_ = ti;
			printf("unknown error\n");
			break;
		}
	}
}
math::vec3 Quadrotor::angular_accel_to_torque(int ti, math::vec3 od) {

	math::vec3 torque = 
		I_ * od + 
		omega(ti).cross(I_ * omega(ti));

	return torque;
}
math::vec4	Quadrotor::thrust_torque_to_motor_speed(int i, double const & thrust, math::vec3 const & torque) {

	math::vec4 temp(torque);

	plant_->gamma1_[i] = A4inv_ * temp;

	// thrust
	plant_->gamma0_[i] = thrust / (k_ * 4.0);

	if(!plant_->gamma1_[i].isSane()) {
		printf("gamma1\n");
		plant_->gamma1_[i].print();
		printf("A4\n");
		A4inv_.print();
		printf("temp\n");
		temp.print();
		throw;
	}

	return (plant_->gamma1_[i] + plant_->gamma0_[i]);
}	
void Quadrotor::write() {
	int n = (ti_f_ > 0) ? ti_f_ : N_;
	
	brain_->write(n);
	//brain_->pos_->write(ti_f_);
	//brain_->att_->write(ti_f_);
	plant_->write(n);
	telem_->write(n);

	write_param();
}
void Quadrotor::write_param() {
	//brain_->att_->write_param();
	//brain_->pos_->write_param();
}

void product(int choices, int repeat, int*& arr, int level) {

	int len = pow(choices, repeat);

	if(level == 0) {
		arr = new int[len * repeat];
	}

	int mod = pow(choices, level);

	for(int a = 0; a < len; a++) {
		int b = (a - (a % mod))/mod % choices;
		//printf("%i ", b);
		arr[a*repeat + level] = b;
	}
	//printf("\n");

	if(level < (repeat-1)) {
		product(choices, repeat, arr, level + 1);
	}

	/*
	   if(level == 0) {
	   for(int a = 0; a < len; a++) {
	   for(int b = 0; b < repeat; b++) {
	   printf("%i ",arr[a*repeat + b]);
	   }
	   printf("\n");
	   }
	   }
	   */
}



