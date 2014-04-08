#include <stdio.h>
#include <cstdlib>
#include <string.h>

#define _DEBUG 0

#include <math/vec3.h>

#include <quadrotor/attitude.h>
#include <quadrotor/brain.h>
#include <quadrotor/position.h>
#include <quadrotor/quadrotor.h>


math::vec3 sinewave(double t) {
	return math::vec3(sin(t * 4.0 / M_PI),t,0.0);
}

void set_coeff(double* center, double* length, int choices, int repeat, double* coeff) {
	
	for(int r = 0; r < repeat; r++) {
		for(int c = 0; c < choices; c++) {
			coeff[r*choices + c] = center[r] + length[r] * (c * 2.0 / (choices - 1.0) - 1.0);
		}
	}
}

void reset_quadrotor(Quadrotor* r, double* C) {
	r->reset();
	

	r->brain_->pos_->C1_.SetDiagonal(C[0], C[0], C[0]);
	r->brain_->pos_->C2_.SetDiagonal(C[1], C[1], C[1]);
	r->brain_->pos_->C4_.SetDiagonal(C[2], C[2], C[2]);

	r->brain_->att_->C1_.SetDiagonal(C[3], C[3], C[3]);
	r->brain_->att_->C2_.SetDiagonal(C[4], C[4], C[4]);

	r->brain_->objs_.push_back(
			new Command::Move(math::vec3(1,0,0), math::vec3(0.01,0.01,0.01))
			);
	
	
}

void sub2(Quadrotor* r, double* C, double& ts, int& N, int a, int& b) {
	//
	
	//Quadrotor* r = new Quadrotor(0.01, N);
	
	reset_quadrotor(r, C);

	r->ti_stop_ = N;

	r->run();
	
	Command::Move* move = (Command::Move*)(r->brain_->pos_->pos_);

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

void print_arr(double* arr, int len) {
	for(int a = 0; a < len; a++) {
		printf("%f ", arr[a]);
	}
	printf("\n");
}

void map() {
	int* arr;
	

	double dt = 0.01;
	int N = 10000;
	Quadrotor* r = new Quadrotor(dt, N);
	
	int choices = 5;
	int repeat = 5;
	//int len = pow(choices, repeat);
	
	product(choices, repeat, arr);
	
	//double center[] = {10.0,  3.0,  7.0,  3.0};
	//double length[] = { 9.9,  2.9,  6.9,  2.9};
	
	double center[] = {10.0,  10.0,  10.0,  10.0, 10.0};
	double length[] = {9.9,  9.9,  9.9,  9.9, 9.9};
	
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


void normal(int N) {

	double dt = 0.01;

	Quadrotor* r = new Quadrotor(dt, N);

	r->brain_->pos_->read_param();
	r->brain_->att_->read_param();
	
	//r->brain_->objs_.push_back(new Command::Move(math::vec3(1,0,0)));
	r->brain_->objs_.push_back(new Command::Move(math::vec3(5,0,0), math::vec3(0.01,0.01,0.01)));
	//r->brain_->objs_.push_back(new Command::Move(math::vec3(1,1,0), math::vec3(0.01,0.01,0.01)));
	//r->brain_->objs_.push_back(new Command::Path(sinewave));
	
	r->run();

	r->write();
}

int main(int argc, const char ** argv) {

	//int N = atoi(argv[1]);

	printf("%i\n",(int)sizeof(math::vec3));

	if(argc != 2) {
		printf("usage: %s <mode>\n",argv[0]);
		exit(0);
	}

	if(strcmp(argv[1],"n")==0) {
		normal(2000);
	} else if(strcmp(argv[1],"m")==0) {
		map();
	} else {
		printf("invalid mode\n");
	}

	//b->att_->write();
}



