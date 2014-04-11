
#include <quadrotor/plant.h>
#include <quadrotor/quadrotor.h>
#include <quadrotor/ControlLaw/ControlLaw.h>

double coeff(double* r, int n, int i, int k) {
	
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

CL::Base::Base(Quadrotor* r): r_(r) {}

void	CL::Thrust::Step(int i, double h) {
	
	r_->plant_->gamma0_[i] = thrust_[i] / (r_->k_ * 4.0);
	
}
void	CL::Alpha::Step(int i, double h) {
	
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


//math::vec3 torque = r_->angular_accel_to_torque(i, alpha_[i]);

void	CL::Terms::set_poles(double* p, int n) {
	
	double c[5];
	
	for(int i = 0; i < n; ++i) {
		c[i] = coeff(p, n, 0, i);
		c_[i].SetDiagonal(c[i], c[i], c[i]);
	}
/*	
	double C1 = coeff(p, 5, 0, 0);
	double C2 = coeff(p, 5, 0, 1);
	double C3 = coeff(p, 5, 0, 2);
	double C4 = coeff(p, 5, 0, 3);
	double C5 = coeff(p, 5, 0, 4);

//	printf("poles % e % e % e % e % e\n",p[0],p[1],p[2],p[3],p[4]);
//	printf("coeff % e % e % e % e % e\n",C1,C2,C3,C4,C5);

	C1_.SetDiagonal(C1,C1,C1);
	C2_.SetDiagonal(C2,C2,C2);
	C3_.SetDiagonal(C3,C3,C3);
	C4_.SetDiagonal(C4,C4,C4);
	C5_.SetDiagonal(C5,C5,C5);
*/
}

void	CL::X::alloc(int n) {
	for(int i = 0; i < 5; ++i) {
		x_ref_[i].alloc(n);
	}
}
void	CL::V::alloc(int n) {
	for(int i = 0; i < 5; ++i) {
		v_ref_[i].alloc(n);
	}
}
void	CL::Q::alloc(int n) {
	q_ref_.alloc(n);
	for(int i = 0; i < 4; ++i) {
		q_ref__[i].alloc(n);
	}
}
void	CL::Omega::alloc(int n) {
	for(int i = 0; i < 5; ++i) {
		omega_ref_[i].alloc(n);
	}
}

void	CL::Terms::alloc(int n) {
	for(int i = 0; i < 5; ++i) {
		e_[i].alloc(n);
	}
}
void	CL::Terms::write() {
	const char * name = "param.txt";
	FILE* file = fopen(name,"w");
	if(file != NULL) {
		for(int i = 0; i < 5; ++i) {
			c_[i].write(file);
		}

		printf("write file %s\n",name);

		fclose(file);
	}
}
void	CL::Terms::read() {
	const char * name = "param.txt";
	FILE* file = fopen(name,"r");
	if(file != NULL) {
		for(int i = 0; i < 5; ++i) {
			c_[i].read(file);
		}

		printf("read file %s\n",name);

		fclose(file);
	} else {
		printf("no file %s\n", name);
	}
}



