#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

#include <math/mat33.h>

#include <quadrotor/Input.hpp>
#include <quadrotor/quadrotor.h>
#include <quadrotor/array.h>
#include <quadrotor/command.h>

double coeff(double* r, int n, int i, int k);
/*
*/


class Quadrotor;

namespace Command {
	class Base;
	class X;
}

namespace CL {
	class Base {
		public:
			Base(Quadrotor* r);

			//virtual void	SetCommand(int, Command::Base*) = 0;

			virtual void	Step(int, double) = 0;
			virtual bool	Check(int, math::vec3) = 0;
			virtual void	alloc(int) = 0;
			virtual void	write(int) = 0;
			virtual void	Init(int) {}
		public:
			Quadrotor*	r_;

			Command::Base*	command_;
	};
	template <int N> class Terms {
		public:
			virtual	~Terms() {}
			void	set_poles(int* i, double* p, int n) {
				for(int j = 0; j < N; ++j) {
					p_[j] = p[i[j]];
				}
				
				set_coeff();
			}
			void	set_coeff() {
				
				double c[N];
				
				for(int i = 0; i < N; ++i) {
					c[i] = coeff(p_, N, 0, i);
					c_[i].SetDiagonal(c[i], c[i], c[i]);
					printf("c[%i] = %lf\n",i,c[i]);
				}
				
				

				//	printf("poles % e % e % e % e % e\n",p[0],p[1],p[2],p[3],p[4]);
				//	printf("coeff % e % e % e % e % e\n",C1,C2,C3,C4,C5);
			}
			void	write(int n, FILE* file) {
				for(int i = 0; i < N; ++i) {
					e_[i].write(file,n);
				}
			}
			void	write_param() {
				const char * name = "param/terms.txt";
				FILE* file = fopen(name,"w");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						c_[i].write(file);
					}
					fwrite(p_, sizeof(double), N, file);

					printf("write file %s\n",name);

					fclose(file);
				}
			}
			void	read_param() {
				const char * name = "param/terms.txt";
				FILE* file = fopen(name,"r");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						c_[i].read(file);
					}
					fwrite(p_, sizeof(double), N, file);

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
			double			p_[N];
	};
	template <int N> class X: virtual public Base, public Terms<N> {
		public:
			X(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual bool	Check(int, math::vec3) = 0;
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
			virtual void	Init(int i) {
				printf("%s i=%i\n",__PRETTY_FUNCTION__,i);
				Command::X* x = dynamic_cast<Command::X*>(command_);

				// back fill
				x_ref_[0][i] = x->in_->f(r_->t(i));
				x_ref_[0][i-1] = x->in_->f(r_->t(i-1));
				x_ref_[0][i-2] = x->in_->f(r_->t(i-2));
				x_ref_[0][i-3] = x->in_->f(r_->t(i-3));
			}
		public:
			Array<math::vec3>	x_ref_[N];
	};
	template <int N> class V: virtual public Base, public Terms<N> {
		public:
			V(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual bool	Check(int, math::vec3) = 0;
			virtual void	alloc(int n) {
				for(int i = 0; i < N; ++i) {
					v_ref_[i].alloc(n);
				}
				Terms<N>::alloc(n);
			}
			virtual void	write(int n) {
				printf("%s %i\n",__PRETTY_FUNCTION__,n);
				FILE* file = fopen("data/cl_v.txt","w");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						v_ref_[i].write(file, n);
					}

					Terms<N>::write(n, file);

					fclose(file);
				} else {
					printf("file error\n");
				}


			}
		public:
			Array<math::vec3>	v_ref_[N];
	};
	template <int N> class Q: virtual public Base, public Terms<N+1> {
		public:
			Q(Quadrotor* r): Base(r) {
				alloc(r_->N_);
			}
			virtual void	write(int n) {
				printf("%s %i\n",__PRETTY_FUNCTION__,n);
				FILE* file = fopen("data/cl_q.txt","w");
				if(file != NULL) {
					q_ref_.write(file, n);
					for(int i = 0; i < N; ++i) {
						q_ref__[i].write(file, n);
					}

					Terms<N+1>::write(n, file);

					fclose(file);
				} else {
					printf("file error\n");
				}


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
			virtual void	write(int n) {
				printf("%s %i\n",__PRETTY_FUNCTION__,n);
				FILE* file = fopen("data/cl_omega.txt","w");
				if(file != NULL) {
					for(int i = 0; i < N; ++i) {
						omega_ref_[i].write(file, n);
					}

					Terms<N>::write(n, file);

					fclose(file);
				} else {
					printf("file error\n");
				}


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
