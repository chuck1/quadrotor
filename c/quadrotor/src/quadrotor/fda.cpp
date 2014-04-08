
#include <math/vec3.h>
#include <math/quat.h>

#include <quadrotor/fda.h>

void zero(math::vec3& a) {
	a.LoadZero();
}
void zero(double& a) {
	a = 0.0;
}
bool sane(math::vec3 const & a) {
	return a.isSane();
}
bool sane(double const & a) {
	if(isnan(a)) return false;
	if(isinf(a)) return false;
	return true;
}
void print(math::vec3 const & a) {
	a.print();
}
void print(double const & a) {
	printf("%lf\n",a);
}


math::quat diff(math::quat const & a, math::quat const & b) {
	return (a * b.getConjugate());
}



void forward_quavec(Array<math::quat> q, Array<math::vec3> qd, double h, int ti, int ti_0, int pre, double tc/* = 0.0*/) {
	
	if (ti_0 > (pre + 1)) {
		math::quat r = diff(q[ti], q[ti-1]);
		
		//qd[ti] = r.getOmega(h) * 2.0 - qd[ti-1];
		qd[ti] = r.getOmega(h);
	} else if(ti_0 > (pre + 0)) {
		math::quat r = diff(q[ti], q[ti-1]);
		
		qd[ti] = r.getOmega(h);
	}
}




