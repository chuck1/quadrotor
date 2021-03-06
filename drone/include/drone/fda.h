#ifndef __FDA__
#define __FDA__

#include <glm/glm.hpp>

#include <drone/util/print.hpp>
#include <drone/array.h>


void zero(glm::vec3& a);
void zero(double& a);

bool sane(glm::vec3 const & a);
bool sane(double const & a);


glm::quat diff(glm::quat const & a, glm::quat const & b);

// a low pass filter???
template<typename T>
void			low_pass(
		Array<T> & x,
		Array<T> & y,
		int ti,
		double dt,
		double tc)
{
	if(ti > 3) {
		//v[ti] = v[ti-2] * 0.25 + v[ti-1] * 0.5 + v[ti] * 0.25;
		
		y[ti] =  (y[ti-1] * tc + x[ti] * dt) / (tc + dt);
	} else if(ti > 0) {
		y[ti] =  (y[ti-1] * tc + x[ti] * dt) / (tc + dt);
	}
}

/**
 * use forward differencing to calculate the derivative
 * of v at time step i
 * h = t(i) - t(i-1)
 */
template<typename T>
void			forward(
		Array<T> & v,
		Array<T> & vd,
		float h,
		int i)
{
	// instead of checking to see if previous steps are
	// available, time series data will be back-filled on initialization

	vd[i] = ((v[i] - v[i-1]) / h + vd[i-1]) / 2.0f;
	
	//vd[ti] = vd[ti-1] + (v[ti] - v[ti-1] * 2.0 + v[ti-2]) / (2.0 * h);

	//vd[ti] = (v[ti] - v[ti-1]) / h;

	if(!sane(vd[i])) {
		printf("forward insane\n");
		printf("dt %lf\n",h);
		printf("vd[i]\n");
		drone::print(vd[i]);
		printf("v[i-1]\n");
		drone::print(v[i-1]);
		printf("v[i]\n");
		drone::print(v[i]);
		throw;
	}
}
void forward_quavec(Array<glm::quat> v, Array<glm::vec3> vd, double h, int i);

#endif
