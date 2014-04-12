#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <memory>

#define _DEBUG 0

#include <math/vec3.h>

#include <quadrotor/attitude.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/ControlLaw/ControlLaw.h>
#include <quadrotor/ControlLaw/Jounce.h>

void print_arr(double* arr, int len) {
	for(int a = 0; a < len; a++) {
		printf("%f ", arr[a]);
	}
	printf("\n");
}
math::vec3 sinewave(double t) {
	
	double per = 15.0;
	
	return math::vec3(sin(t * 2.0 * M_PI / per), t, 0.0);
}

void set_coeff(double* center, double* length, int choices, int repeat, double* coeff) {
	
	for(int r = 0; r < repeat; r++) {
		for(int c = 0; c < choices; c++) {
			coeff[r*choices + c] = center[r] + length[r] * (c * 2.0 / (choices - 1.0) - 1.0);
		}
	}
}

class search {
	public:
		search();
		void		step();
		void		exec(int);
		bool		test();

		Quadrotor*	r_;
		double		C_[5];
		double		ts_;
		int		current_;
		int		n_;
		int		count_;
};
search::search():
	ts_(1e10),
	current_(0),
	n_(10000),
	count_(0)
{
	r_ = new Quadrotor(0.01, n_);

	C_[0] = -0.9;
	C_[1] = -0.9;
	C_[2] = -0.8;
	C_[3] = -0.7;
	C_[4] = -0.7;
}
void search::step() {
	
	double low  = C_[current_] / 1.1;
	double val  = C_[current_];
	double high = C_[current_] * 1.1;
	
	C_[current_] = low;
	if(test()) {
		count_ = 0;
		printf("%i %lf %lf\n", current_, C_[current_], ts_);
		//print_arr(C_,5);
		return;
	}

	C_[current_] = high;
	if(test()) {
		count_ = 0;
		printf("%i %lf %lf\n", current_, C_[current_], ts_);
		//print_arr(C_,5);
		return;
	}
	
	count_++;
	
	printf("no change\n");
	
	C_[current_] = val;
	//print_arr(C_,5);
	current_ = (current_ + 1) % 5;
}
void search::exec(int m) {

	if(test()) {
		for(int i = 0; i < m; i++) {
			step();

			if(count_ == 5) {
				break;
			}
		}
		print_arr(C_,5);

		n_ += 100;
		test();
		r_->write();
	} else {
		printf("failed\n");
	}
}

math::vec3 constant(double) {
	return math::vec3(1,0,0);
}

bool search::test() {
	r_->reset();

	Jounce::X* x = dynamic_cast<Jounce::X*>(r_->brain_->cl_x_);
	
	x->set_poles(C_, 5);
	r_->brain_->objs_.push_back(
			new Command::X(r_, constant, math::vec3(0.01,0.01,0.01))
			);
	r_->ti_stop_ = n_;
	r_->run();

	Command::X* move = (Command::X*)(r_->brain_->obj_);

	if(move->flag_ & Command::Base::Flag::COMPLETE) {
		if(move->ts_ < ts_) {
			ts_ = move->ts_;
			n_ = move->ti_s_;
			return true;
		}
	}
	return false;
}


void reset_quadrotor(Quadrotor* r, double* C) {
	r->reset();

	/*
	   r->brain_->pos_->C1_.SetDiagonal(C[4], C[4], C[4]);
	   r->brain_->pos_->C2_.SetDiagonal(C[0], C[0], C[0]);
	   r->brain_->pos_->C3_.SetDiagonal(C[1], C[1], C[1]);
	   r->brain_->pos_->C4_.SetDiagonal(C[2], C[2], C[2]);
	   r->brain_->pos_->C5_.SetDiagonal(C[3], C[3], C[3]);
	   */
	Jounce::X* x = dynamic_cast<Jounce::X*>(r->brain_->cl_x_);
	x->set_poles(C, 5);

	//r->brain_->att_->C1_.SetDiagonal(C[3], C[3], C[3]);
	//r->brain_->att_->C2_.SetDiagonal(C[4], C[4], C[4]);

	r->brain_->objs_.push_back(
			new Command::X(r, constant, math::vec3(0.01,0.01,0.01))
			);


}

void sub2(Quadrotor* r, double* C, double& ts, int& N, int a, int& b) {
	//

	//Quadrotor* r = new Quadrotor(0.01, N);

	reset_quadrotor(r, C);

	r->ti_stop_ = N;

	r->run();

	Command::X* move = (Command::X*)(r->brain_->obj_);

	if(move->flag_ & Command::Base::Flag::COMPLETE) {
		if(move->ts_ < ts) {
			ts = move->ts_;
			N = move->ti_s_;
			b = a;

			printf("b %i ts %f\n", b, move->ts_);

			r->write();
			r->write_param();
		}
	}
}

void set_C(double* coeff, int* arr, int choices, int repeat, int a, double* C) {
	for(int c = 0; c < repeat; c++) {
		C[c] = coeff[c*choices + arr[a*repeat + c]];
	}
}

int sub1(Quadrotor* r, int* arr, double* coeff, int choices, int repeat) {

	int len = pow(choices, repeat);

	double C[repeat];

	int N = 10000;

	//Quadrotor* r = new Quadrotor(0.01, N);

	int b = -1;
	double ts = 1e10;

	// previous winner
	set_C(coeff, arr, choices, repeat, (len-1)/2, C);
	sub2(r, C, ts, N, (len-1)/2, b);

	for(int a = 0; a < len; a++) {
		set_C(coeff, arr, choices, repeat, a, C);
		//printf("%i %f %f %f %f %f\n",a,C[0],C[1],C[2],C[3],C[4]);

		sub2(r, C, ts, N, a, b);

	}

	return b;
}



void map() {
	int* arr;


	double dt = 0.01;
	int N = 100000;
	Quadrotor* r = new Quadrotor(dt, N);

	int choices = 11;
	int repeat = 4;
	//int len = pow(choices, repeat);

	product(choices, repeat, arr);

	//double center[] = {10.0,  3.0,  7.0,  3.0};
	//double length[] = { 9.9,  2.9,  6.9,  2.9};

	double center[] = {-0.5,  -0.5,  -0.5,  -0.5};
	double length[] = { 0.5,   0.5,   0.5,   0.5};

	double* coeff = new double[choices * repeat];

	printf("map start\n");

	for(int b = 0; b < 20; b++) {
		int a = -1;

		set_coeff(center, length, choices, repeat, coeff);
		a = sub1(r, arr, coeff, choices, repeat);

		if(a == -1) {
			printf("failed\n");
			return;
		}	

		for(int c = 0; c < repeat; c++) {
			center[c] = coeff[c*choices + arr[a*repeat + c]];
			length[c] = length[c] * 0.8;
		}



		printf("center length\n");
		print_arr(center, repeat);
		print_arr(length, repeat);

		//printf("a %i\n",a);
	}



}


void normal(int N, double dt) {

	Quadrotor* r = new Quadrotor(dt, N);

	Jounce::X* x = dynamic_cast<Jounce::X*>(r->brain_->cl_x_);
	//x->read();

	
	double poles[] = {
		-0.9,
		-0.9,
		-0.7,
		-0.5,
		-0.5};

	x->set_poles(poles, 5);

	r->brain_->objs_.push_back(new Command::X(r, constant));

	//r->brain_->objs_.push_back(new Command::Move(math::vec3(0.01,0,0)));
	//r->brain_->objs_.push_back(new Command::Move(math::vec3(1,0,0), math::vec3(0.01,0.01,0.01)));
	//r->brain_->objs_.push_back(new Command::Move(math::vec3(1,1,0), math::vec3(0.01,0.01,0.01)));
	//r->brain_->objs_.push_back(new Command::Path(sinewave));

	r->run();

	r->write();
}
void srch() {

	search s;
	s.exec(200);

}
int main(int argc, const char ** argv) {
	
	double dt = 0.01;
	
	//int N = atoi(argv[1]);
	
	printf("%i\n",(int)sizeof(math::vec3));

	if(argc != 2) {
		printf("usage: %s <mode>\n",argv[0]);
		exit(0);
	}

	if(strcmp(argv[1],"n")==0) {
		normal(10000, dt);
	} else if(strcmp(argv[1],"m")==0) {
		map();
	} else if(strcmp(argv[1],"s")==0) {
		srch();
	} else {
		printf("invalid mode\n");
	}

	//b->att_->write();
}



