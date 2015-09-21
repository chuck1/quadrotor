#include <cmath>
#include <typeinfo>

#include <glm/gtc/quaternion.hpp>

//#include <drone/attitude.h>
//#include <drone/position.h>
#include <drone/Brain.hpp>
#include <drone/Telem.hpp>
#include <drone/Plant.hpp>
#include <drone/Drone.hpp>
#include <drone/command/command.h>
#include <drone/command/Stop.hpp>

Drone::Drone(/*float dt,*/ int N):
	_M_flag(0),
	_M_stop_cause(Drone::StopCause::TIME_STEP),
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

	// where do these come from???
	Kv_	= 1450;	// back EMF (RPM / V)
	Kv_ = 1.0 / Kv_ * 0.1047;

	Kt_	= Kv_;
	Ktau_	= 0.5;

	k_ = Kv_ * Ktau_ * sqrt(rho_ * Asw_);
	b_ = 0.5 * pow(R_,3) * rho_ * CD_ * A_;
	
	//I_ = ;
	Iinv_ = glm::inverse(I_);
	
	P_max_		= 340.0;

	// where does this come from???
	gamma_max_	= pow(P_max_ * sqrt(2.0 * rho_ * Asw_) / pow(k_, 3.0/2.0), 2.0/3.0);
	//gamma_max_ = 1e10;
	

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
}
std::shared_ptr<Command::Base>		Drone::get_command()
{
	assert(brain_);
	return brain_->get_obj();
}
void			Drone::init()
{
	telem_ = new Telem(shared_from_this());
	//plant_ = new Plant(shared_from_this());
	brain_ = new Brain(shared_from_this());

	gamma0_.alloc(10);
	gamma1_.alloc(10);

}
glm::vec3&		Drone::x(int i) { return telem_->x_[i]; }
glm::vec3&		Drone::v(int i) { return telem_->v_[i]; }
glm::vec3&		Drone::a(int i) { return telem_->a_[i]; }
glm::vec3&		Drone::jerk(int i) { return telem_->jerk_[i]; }
//glm::vec3&		Drone::jounce(int i) { return telem_->jounce_[i]; }
glm::quat&		Drone::q(int i) { return telem_->q_[i]; }
glm::vec3&		Drone::omega(int i) { return telem_->omega_[i]; }
glm::vec3&		Drone::alpha(int i) { return telem_->alpha_[i]; }
void			Drone::reset()
{
	ti_f_ = 0;

	brain_->reset();
}
void			Drone::step(float dt)
{
	// advance time
	t_[_M_i] = t_[_M_i-1] + dt;

	try {
		brain_->step(_M_i-1, dt);
		//plant_->step(_M_i-1);
		telem_->step(_M_i, dt);
	} catch(EmptyQueue &e) {
		//printf("empty queue ti\n");
		ti_f_ = _M_i;

		_M_stop_cause = Drone::StopCause::OBJ;

		throw 0;
	} catch(StopCond &e) {
		//printf("stop cond occured\n");
		//printf("%s\n", e.what());
		//printf("%s\n", typeid(e).name());
		ti_f_ = _M_i;

		_M_stop_cause = Drone::StopCause::INF;

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
void			Drone::run(float dt)
{
	//printf("dt %f\n", dt_);

	//printf("command queue %i\n", (int)brain_->objs_.size());

	_M_stop_cause = Drone::StopCause::TIME_STEP;

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
glm::vec3 Drone::angular_accel_to_torque(int ti, glm::vec3 alpha)
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
int	index_abs_max(glm::vec4 const & x)
{
	auto y = glm::abs(x);

	int i = 0;
	int v = y[0];

	for(int j = 1; j < 4; ++j) {
		if(y[j] > v) {
			i = j;
			v = y[j];
		}
	}

	return i;
}
float		sign(float f)
{
	if(f >= 0) return 1;
	return -1;
}
glm::vec4		Drone::thrust_torque_to_motor_speed(
		int i,
		float const & thrust,
		glm::vec3 const & torque)
{
	// thrust
	float g0 = thrust / (k_ * 4.0);

	// torque
	glm::vec4 temp(0,torque);

	auto g1 = A4inv_ * temp;

	// check
	//bool c0 = glm::any(glm::isnan(plant_->gamma1_[i]));
	//bool c1 = glm::any(glm::isinf(plant_->gamma1_[i]));
	bool c0 = glm::any(glm::isnan(gamma1_[i]));
	bool c1 = glm::any(glm::isinf(gamma1_[i]));
	if(c0 || c1) {
		printf("gamma1\n");
		//::print(plant_->gamma1_[i]);
		printf("A4\n");
		//A4inv_.print();
		printf("temp\n");
		//temp.print();
		throw;
	}

	// TODO figure out how to scale intermediate variable (snap) instead

	auto g = g0 + g1;

	// scale
	
	auto g_abs = glm::abs(g);

	// if g0 exceeds gamma max
	if(abs(g0) > gamma_max_) {
		// eliminate torque
		g1 = glm::vec4(0);
		// scale thrust
		g0 = sign(g0) * gamma_max_;
	} else {

		int i = index_abs_max(g);

		float x = g[i];

		if(abs(x) > gamma_max_) {

			float xs = sign(x) * gamma_max_ - g0;

			float scale = xs / x;

			assert(scale>0);

			g1 = g1 * scale;
		}
	}

	// after scaling
	g = g0 + g1;

	//plant_->gamma0_[i] = g0;
	//plant_->gamma1_[i] = g1;
	gamma0_[i] = g0;
	gamma1_[i] = g1;

	return g;
}	
void			Drone::write()
{
	int n = (ti_f_ > 0) ? ti_f_ : N_;

	brain_->write(n);
	//brain_->pos_->write(ti_f_);
	//brain_->att_->write(ti_f_);
	//plant_->write(n);
	telem_->write(n);

	write_param();
}
void			Drone::write_param()
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
bool	Drone::isset_debug() const
{
	unsigned long ret = _M_flag & Drone::Flag::DEBUG;
	//printf("%lu\n", ret);
	return (ret);
}
float	Drone::get_score(
		std::shared_ptr<Command::Base> cmd,
		std::function<float(Drone*)> metric,
		float score,
		int & N)
{
	printf("brain_->objs_.length() = %lu\n", brain_->objs_.size());

	assert(cmd);
	
	//assert(brain_->obj_);
	auto move = std::dynamic_pointer_cast<Command::X>(cmd);
	assert(move);
	
	//auto stop_x = dynamic_cast<Command::Stop::XSettle*>(move->stop_[0]);
	
	//printf("stop cause %i\n", drone->_M_stop_cause);
	
	float temp_score;
	
	if(!move->stop_.empty()) {
		// a stop object exists
		auto stop_x = move->stop_[0];
		assert(stop_x);
		
		if(move->flag_ & Command::Base::Flag::COMPLETE) {
			// objective complete
			
			temp_score = stop_x->stats_.t_;

			if(temp_score < score) {
				// new maximum simulation steps
				N = stop_x->stats_.i_;
			}

			return temp_score;
		}
	}

	if(_M_stop_cause == Drone::StopCause::TIME_STEP) {
		float m = metric(this);
		printf("r->t(%i) = %f\n", N, t(N));
		printf("metric   = %f\n", m);
		return t(N) + m;
	}

	if (_M_stop_cause == Drone::StopCause::INF) {
		return score + 1;
	}

	assert(0);
	return 0;
}
std::shared_ptr<CL::Base>	Drone::get_cl()
{
	assert(brain_);

	return brain_->get_cl();
}






