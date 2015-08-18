#ifndef __CONTROL_LAW__
#define __CONTROL_LAW__

#include <glm/glm.hpp>

#include <drone/util/decl.hpp>
#include <drone/command/Input.hpp>
#include <drone/Drone.hpp>
#include <drone/array.h>
#include <drone/command/command.h>

float coeff(float* r, int n, int i, int k);
/*
*/

namespace CL {
	class Base
	{
		public:
			Base();

			//virtual void	SetCommand(int, Command::Base*) = 0;

			virtual void	init();
			virtual void	step(int, float) = 0;
			virtual bool	check(int, glm::vec3) = 0;
			virtual void	alloc(int) = 0;
			virtual void	write(int) = 0;
		public:
			Quadrotor*	r_;

			Command::Base*	command_;
	};
	template <int N> class Terms {
		public:
			virtual	~Terms() {}
			void		set_poles(int* i, float* p, int n)
			{
				for(int j = 0; j < N; ++j) {
					p_[j] = p[i[j]];
				}
				
				set_coeff();
			}
			void	set_coeff() {
				
				float c[N];
				
				for(int i = 0; i < N; ++i) {
					c[i] = coeff(p_, N, 0, i);
					
					c_[i][0][0] = c[i];
					c_[i][1][1] = c[i];
					c_[i][2][2] = c[i];
					//.SetDiagonal(c[i], c[i], c[i]);
					
					//printf("c[%i] = %lf\n",i,c[i]);
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
					fwrite(p_, sizeof(float), N, file);

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
					fwrite(p_, sizeof(float), N, file);

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

			glm::mat3		c_[N];
			Array<glm::vec3>	e_[N];
			float			p_[N];
	};
	template <int N>
	class X:
		virtual public Base,
		public Terms<N>
	{
		public:
			X()
			{
			//	alloc(r_->N_);
			}
			virtual bool	check(int, glm::vec3) = 0;
			virtual void	alloc(int n) {
				//printf("%s\n",__PRETTY_FUNCTION__);
				for(int i = 0; i < N; ++i) {
					x_ref_[i].alloc(n);
				}
				Terms<N>::alloc(n);
			}
			virtual void	write(int n) {
				//printf("%s %i\n",__PRETTY_FUNCTION__,n);
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
			virtual void	init(int i)
			{
				//printf("%s i=%i\n",__PRETTY_FUNCTION__,i);
				Command::X* x = dynamic_cast<Command::X*>(command_);

				// back fill
				x_ref_[0][i] = x->in_->f(r_->t(i));
				x_ref_[0][i-1] = x->in_->f(r_->t(i-1));
				x_ref_[0][i-2] = x->in_->f(r_->t(i-2));
				x_ref_[0][i-3] = x->in_->f(r_->t(i-3));
			}
		public:
			Array<glm::vec3>	x_ref_[N];
	};
	template <int N> class V: virtual public Base, public Terms<N> {
		public:
			V()
			{
			//	alloc(r_->N_);
			}
			virtual bool	check(int, glm::vec3) = 0;
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
			Array<glm::vec3>	v_ref_[N];
	};
	template <int N> class Q: virtual public Base, public Terms<N+1> {
		public:
			Q()
			{
			//	alloc(r_->N_);
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
			Array<glm::quat>	q_ref_;
			Array<glm::vec3>	q_ref__[N];
	};
	template <int N>
	class Omega:
		virtual public Base,
		public Terms<N>
	{
		public:
			//Omega(): r_(0) {}
			void	init()
			{
				assert(r_);
				alloc(r_->N_);
			}
			virtual void	alloc(int n)
			{
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
			//void		set_ref(int i, glm::vec3 omega) { omega_ref_[i] = omega; }
		public:
			Array<glm::vec3>	omega_ref_[2];
	};


	class Thrust: virtual public CL::Base {
		public:
			virtual void		step(int, float);
			virtual void		alloc(int);
			virtual void		write(int);
		public:
			Array<float>		thrust_;
	};
	class Alpha: virtual public CL::Base {
		public:
			virtual void		step(int, float);
			virtual void		alloc(int);
			virtual void		write(int);
		public:
			Array<glm::vec3>	alpha_;
	};
}












#endif
