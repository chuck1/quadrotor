
#include <quadrotor/plant.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/ControlLaw/ControlLaw.h>



CL::Base::Base(Quadrotor* r): r_(r) {}

void	CL::Thrust::Step(int i, double h) {
	//printf("%s\n",__PRETTY_FUNCTION__);

	r_->plant_->gamma0_[i] = thrust_[i] / (r_->k_ * 4.0);
	
}
void	CL::Alpha::Step(int i, double h) {
	//printf("%s\n",__PRETTY_FUNCTION__);

	math::vec3 torque = r_->angular_accel_to_torque(i, alpha_[i]);
	
	math::vec4 temp(torque);

	r_->plant_->gamma1_[i] = r_->A4inv_ * temp;
/*
	if(!quad_->plant_->gamma1_[ti].isSane()) {
		printf("gamma1\n");
		quad_->plant_->gamma1_[ti].print();
		printf("A4\n");
		quad_->A4inv_.print();
		printf("temp\n");
		temp.print();
		throw;
	}*/
}

void	CL::Alpha::alloc(int n) {
	printf("%s\n",__PRETTY_FUNCTION__);
	alpha_.alloc(n);
}
void	CL::Alpha::write(int n) {
	FILE* file = fopen("data/alpha.txt","w");
	alpha_.write(file,n);
	fclose(file);
}

void	CL::Thrust::alloc(int n) {	
	thrust_.alloc(n);

	double t = r_->m_ * -r_->gravity_.z();
	thrust_[-1] = t;
	thrust_[-2] = t;
}
void	CL::Thrust::write(int n) {
	FILE* file = fopen("data/thrust.txt","w");
	thrust_.write(file,n);
	fclose(file);
}

double coeff(double* r, int n, int i, int k) {
	// coeff -- coefficiencts to a polynomial with n roots stored in r
	// i must start at 0
	// k is the power of x associated with this coeff
	// c[k] == 1
	// do not call this function with k==n
	
	//int i__ = i;

	double c = 0;

	//printf("%i\n",i);

	for(; i <= k; i++) {
		//printf("%i %i\n",i,k);
		if((k+1) < n) {
			//printf("descend\n");
			c += r[i] * coeff(r, n, i+1, k+1);
		} else {
			//printf("stop\n");
			c += r[i];
			//printf("%e %e %i\n",c,r[i],i);
		}
	}

	//if(i__==0) printf("%e\n",-c);

	return -c;
}
