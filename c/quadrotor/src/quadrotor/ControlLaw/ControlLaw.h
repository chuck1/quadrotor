#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

#include <math/mat33.h>

#include <quadrotor/quadrotor.h>
#include <quadrotor/array.h>

double coeff(double* r, int n, int i, int k);
/*
*/


class Quadrotor;

namespace Command {
	class Base;
}

namespace CL {
	class Base {
		public:
			Base(Quadrotor* r);

			//virtual void	SetCommand(int, Command::Base*) = 0;

			virtual void	Step(int, double) = 0;
			virtual void	alloc(int) = 0;
			virtual void	write(int) = 0;
		public:
			Quadrotor*	r_;

			Command::Base*	command_;
	};
	template <int N> class Terms {
		public:
			virtual	~Terms() {}
			void	set_poles(double* p, int n) {

				if(n != N) {
					printf("error poles\n");
					abort();
				}

				double c[N];

				for(int i = 0; i < N; ++i) {

					c[i] = coeff(p, n, 0, i);
					//c[i] = coeff(p, n, 0, N-1-i);
					
					c_[i].SetDiagonal(c[i], c[i], c[i]);
					printf("c[%i] = %lf\n",i,c[i]);
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
			void	write(int n, FILE* file) {
				for(int i = 0; i < N; ++i) {
					e_[i].write(file,n);
				}
			}
			void	write_param() {
				const char * name = "param.txt";
				FILE* file = fopen(name,"w");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						c_[i].write(file);
					}

					printf("write file %s\n",name);

					fclose(file);
				}
			}
			void	read_param() {
				const char * name = "param.txt";
				FILE* file = fopen(name,"r");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						c_[i].read(file);
					}

					printf("read file %s\n",name);

					fclose(file);
				} else {
					printf("no file %s\n", name);
				}
			}
			virtual void	alloc(int n) {
				for(int i = 0; i < N; ++i) {
					e_[i].alloc(n);
				}
			}

			math::mat33		c_[N];
			Array<math::vec3>	e_[N];
	};
	template <int N> class X: virtual public Base, public Terms<N> {
		public:
			X(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual void	alloc(int n) {
				printf("%s\n",__PRETTY_FUNCTION__);
				for(int i = 0; i < N; ++i) {
					x_ref_[i].alloc(n);
				}
				Terms<N>::alloc(n);
			}
			virtual void	write(int n) {
				printf("%s %i\n",__PRETTY_FUNCTION__,n);
				FILE* file = fopen("data/cl_x.txt","w");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						x_ref_[i].write(file, n);
					}
					
					Terms<N>::write(n, file);
					
					fclose(file);
				} else {
					printf("file error\n");
				}
				
				
			}
		public:
			Array<math::vec3>	x_ref_[N];
	};
	template <int N> class V: virtual public Base, public Terms<N> {
		public:
			V(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual void	alloc(int n) {
				for(int i = 0; i < N; ++i) {
					v_ref_[i].alloc(n);
				}
				Terms<N>::alloc(n);
			}
		public:
			Array<math::vec3>	v_ref_[N];
	};
	template <int N> class Q: virtual public Base, public Terms<N+1> {
		public:
			Q(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual void	alloc(int n) {
				q_ref_.alloc(n);
				for(int i = 0; i < N; ++i) {
					q_ref__[i].alloc(n);
				}
				Terms<N+1>::alloc(n);
			}
		public:
			Array<math::quat>	q_ref_;
			Array<math::vec3>	q_ref__[N];
	};
	template <int N> class Omega: virtual public Base, public Terms<N> {
		public:
			Omega(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual void	alloc(int n) {
				for(int i = 0; i < N; ++i) {
					omega_ref_[i].alloc(n);
				}
				Terms<N>::alloc(n);
			}
			//void		set_ref(int i, math::vec3 omega) { omega_ref_[i] = omega; }
		public:
			Array<math::vec3>	omega_ref_[2];
	};


	class Thrust: virtual public CL::Base {
		public:
			Thrust(Quadrotor* r): CL::Base(r) {}

			virtual void		Step(int, double);
			virtual void		alloc(int);
			virtual void		write(int);
		public:
			Array<double>		thrust_;
	};
	class Alpha: virtual public CL::Base {
		public:
			Alpha(Quadrotor* r): CL::Base(r) {}

			virtual void		Step(int, double);
			virtual void		alloc(int);
			virtual void		write(int);
		public:

			Array<math::vec3>	alpha_;
	};
}












#endif
