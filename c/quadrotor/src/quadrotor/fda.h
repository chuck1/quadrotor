#ifndef __FDA__
#define __FDA__

#include <quadrotor/array.h>

#include <math/vec3.h>
#include <math/quat.h>

void zero(math::vec3& a);
void zero(double& a);

bool sane(math::vec3 const & a);
bool sane(double const & a);

void print(math::vec3 const & a);
void print(double const & a);

math::quat diff(math::quat const & a, math::quat const & b);


template<typename T> void low_pass(Array<T> x, Array<T> y, int ti, double dt, double tc) {
	if(ti > 3) {
		//v[ti] = v[ti-2] * 0.25 + v[ti-1] * 0.5 + v[ti] * 0.25;
		
		y[ti] =  (y[ti-1] * tc + x[ti] * dt) / (tc + dt);
	} else if(ti > 0) {
		y[ti] =  (y[ti-1] * tc + x[ti] * dt) / (tc + dt);
	}
}


template<typename T> void forward(Array<T> v, Array<T> vd, double h, int ti, int ti_0, int pre) {
	if(ti_0 > (pre + 1)) {
		//vd[ti] = (v[ti] - v[ti-1]) * 2.0 / h - vd[ti-1];
		
		vd[ti] = ((v[ti] - v[ti-1]) / h + vd[ti-1]) / 2.0;

		//vd[ti] = vd[ti-1] + (v[ti] - v[ti-1] * 2.0 + v[ti-2]) / (2.0 * h);
		
		//vd[ti] = (v[ti] - v[ti-1]) / h;
	} else if(ti_0 > (pre + 0)) {
		vd[ti] = (v[ti] - v[ti-1]) / h;
	} else {
		zero(vd[ti]);
	}
	
	
	if(!sane(vd[ti])) {
		printf("forward insane\n");
		printf("dt %lf\n",h);
		printf("vd[ti]\n");
		print(vd[ti]);
		printf("v[ti-1]\n");
		print(v[ti-1]);
		printf("v[ti]\n");
		print(v[ti]);
		throw;
	}
}



void forward_quavec(Array<math::quat> v, Array<math::vec3> vd, double h, int ti, int ti_0, int pre, double tc = 0.0);

#endif
