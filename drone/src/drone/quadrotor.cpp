#include <typeinfo>

#include <glm/gtc/quaternion.hpp>

//#include <drone/attitude.h>
//#include <drone/position.h>
#include <drone/Brain.hpp>
#include <drone/Telem.hpp>
#include <drone/Plant.hpp>
#include <drone/Drone.hpp>


Quadrotor::Quadrotor(/*float dt,*/ int N):
	_M_flag(0),
	_M_stop_cause(Quadrotor::StopCause::TIME_STEP),
	//dt_(dt),
	N_(N),
	_M_i(1),
	ti_stop_(N),
	ti_f_(0)
{
	//printf("dt %lf\n",dt);
	
	//t_ = new float[N_];
	//for(int ti = 0; ti < N_; ti++) t_[ti] = dt * (float)ti;
	
	t_.alloc(N);
	t_.fill(0);

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
	Iinv_ = glm::inverse(I_);
	
	P_max_		= 340.0;
	//gamma_max_	= pow(P_max_ * sqrt(2.0 * rho_ * Asw_) / pow(k_, 3.0/2.0), 2.0/3.0);
	gamma_max_ = 1e10;
	

	//printf("gamma max %e\n", gamma_max_);

	//printf("I Iinv\n");
	//I_.print();
	//Iinv_.print();

	// matrices
	A4_ = glm::mat4(
			L_ * k_,	0,		-L_ * k_,	0,
			0,		L_ * k_,	0,		-L_ * k_,
			b_,		-b_,		b_,		-b_,
			1.0,		1.0,		1.0,		1.0);
	
	
	A4_ = glm::transpose(A4_);
	
	A4inv_ = glm::inverse(A4_);

	gravity_ = glm::vec3(0,0,-9.81);

	telem_ = new Telem(this);
	plant_ = new Plant(this);
	brain_ = new Brain(this);
}
glm::vec3&		Quadrotor::x(int i) { return telem_->x_[i]; }
glm::vec3&		Quadrotor::v(int i) { return telem_->v_[i]; }
glm::vec3&		Quadrotor::a(int i) { return telem_->a_[i]; }
glm::vec3&		Quadrotor::jerk(int i) { return telem_->jerk_[i]; }
//glm::vec3&		Quadrotor::jounce(int i) { return telem_->jounce_[i]; }
glm::quat&		Quadrotor::q(int i) { return telem_->q_[i]; }
glm::vec3&		Quadrotor::omega(int i) { return telem_->omega_[i]; }
glm::vec3&		Quadrotor::alpha(int i) { return telem_->alpha_[i]; }
void			Quadrotor::reset()
{
	ti_f_ = 0;

	brain_->reset();
}
void			Quadrotor::step(float dt)
{
	// advance time
	t_[_M_i] = t_[_M_i-1] + dt;

	try {
		brain_->step(_M_i-1, dt);
		plant_->step(_M_i-1);
		telem_->step(_M_i, dt);
	} catch(EmptyQueue &e) {
		//printf("empty queue ti\n");
		ti_f_ = _M_i;

		_M_stop_cause = Quadrotor::StopCause::OBJ;

		throw 0;
	} catch(StopCond &e) {
		//printf("stop cond occured\n");
		//printf("%s\n", e.what());
		//printf("%s\n", typeid(e).name());
		ti_f_ = _M_i;

		_M_stop_cause = Quadrotor::StopCause::INF;

		throw 0;
	}
	/*		
	} catch(...) {
	ti_f_ = ti;
	printf("unknown error\n");
	throw 0;
	break;
	}*/

	_M_i++;
}
void			Quadrotor::run(float dt)
{
	//printf("dt %f\n", dt_);

	//printf("command queue %i\n", (int)brain_->objs_.size());

	_M_stop_cause = Quadrotor::StopCause::TIME_STEP;

	while(_M_i < ti_stop_) {
		if ((_M_i % (N_ / 100)) == 0) {
			//printf("%i %f\n",ti,t[ti]);
		}
		try {
			step(dt);
		} catch(...) {
			break;
		}
	}
}
glm::vec3 Quadrotor::angular_accel_to_torque(int ti, glm::vec3 alpha)
{
	glm::vec3 & o = omega(ti);
	glm::vec3 torque = 
		I_ * alpha + 
		glm::cross(o, I_ * o);

	if(isset_debug()) {
		printf("angular accel to torque\n");
		printf("  alpha  %16e%16e%16e\n", alpha.x, alpha.y, alpha.z);
		printf("  omega  %16e%16e%16e\n", o.x, o.y, o.z);
		printf("  torque %16e%16e%16e\n", torque.x, torque.y, torque.z);
	}
	return torque;
}
glm::vec4		Quadrotor::thrust_torque_to_motor_speed(
		int i,
		float const & thrust,
		glm::vec3 const & torque)
{

	glm::vec4 temp(0,torque);

	plant_->gamma1_[i] = A4inv_ * temp;

	// thrust
	plant_->gamma0_[i] = thrust / (k_ * 4.0);

	if(glm::any(glm::isnan(plant_->gamma1_[i])) || glm::any(glm::isinf(plant_->gamma1_[i]))) {
		printf("gamma1\n");
		//::print(plant_->gamma1_[i]);
		printf("A4\n");
		//A4inv_.print();
		printf("temp\n");
		//temp.print();
		throw;
	}

	return (plant_->gamma1_[i] + plant_->gamma0_[i]);
}	
void			Quadrotor::write()
{
	int n = (ti_f_ > 0) ? ti_f_ : N_;

	brain_->write(n);
	//brain_->pos_->write(ti_f_);
	//brain_->att_->write(ti_f_);
	plant_->write(n);
	telem_->write(n);

	write_param();
}
void			Quadrotor::write_param()
{
	//brain_->att_->write_param();
	//brain_->pos_->write_param();
}
void			product(int choices, int repeat, int*& arr, int level)
{

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
bool	Quadrotor::isset_debug() const
{
	unsigned long ret = _M_flag & Quadrotor::Flag::DEBUG;
	//printf("%lu\n", ret);
	return (ret);
}


