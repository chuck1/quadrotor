#ifndef __QUADROTOR_INPUT__
#define __QUADROTOR_INPUT__

#include <stdio.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * phase - phase shift
 */
float fourier_square(float t, float phase, float L, int n);

namespace Input {
	namespace Vec3 {
		class Base {
			public:
				virtual glm::vec3	f(float) = 0;
				int			_M_countdown_zero_derivative;
		};
		class Const: public Input::Vec3::Base {
			public:
				Const(glm::vec3 v): v_(v) {}
				virtual glm::vec3	f(float) { return v_; }
				void		set(glm::vec3 v)
				{
					v_=v;
				}
				glm::vec3	v_;
		};
		class Circle: public Input::Vec3::Base
		{
			public:
				Circle(float r, float T): r_(r), T_(T) {
					omega_ = 2.0 * M_PI / T_;
				}
				virtual glm::vec3	f(float t)
				{
					float x = sin(omega_ * t);
					float y = cos(omega_ * t);
					return (glm::vec3(x, y, 0) * r_);
				}
				/** radius */
				float	r_;
				/** period */
				float	T_;
				float	omega_;
		};
		class Square: public Input::Vec3::Base
		{
		public:
			Square(float r, float T): r_(r), T_(T) {
				omega_ = 2.0 * M_PI / T_;
			}
			virtual glm::vec3	f(float t)
			{
				if(t < 0) t += T_;

				//float temp = 4*t/T_;

				
				float R = sqrt(2)/2*r_;
				
				float index = floor(t/T_);
				float frac = t/T_ - floor(t/T_);

				//float segment_index = floor(temp);
				float segment_index = floor(4*frac);

				//float segment_frac = temp - segment_index;
				float segment_frac = 4*t/T_ - floor(4*t/T_);


				if(0) {
				printf("%12s %12s %12s %12s %12s %12s\n",
						"t","T","index","frac","seg index",
						"seg frac");
				printf("%12f %12f %12f %12f %12f %12f\n",
						t,
						T_,
						index,
						frac,
						segment_index,
						segment_frac);
				}


				float x;	
				float y;
				switch((int)segment_index) {
					case 0:
						x = (-R)*(1-segment_frac) + (R)*(segment_frac);
						y = R;
						break;
					case 1:
						y = (R)*(1-segment_frac) + (-R)*(segment_frac);
						x = R;
						break;
					case 2:
						x = (R)*(1-segment_frac) + (-R)*(segment_frac);
						y = -R;
						break;
					case 3:
						y = (-R)*(1-segment_frac) + (R)*(segment_frac);
						x = -R;
						break;
					default:
						printf("%f %f %f %f %i\n",
								t,
								T_,
								index,
								frac, 
								(int)segment_index);
						assert(0);
						break;
				}

				return (glm::vec3(x, y, 0) * r_);
			}
			/** radius */
			float	r_;
			/** period */
			float	T_;
			float	omega_;
		};
		class SquareFourier: public Input::Vec3::Base
		{
			public:
				SquareFourier(float r, float T): r_(r), T_(T) {
					omega_ = 2.0 * M_PI / T_;
				}
				virtual glm::vec3	f(float t)
				{

					// number of sine waves
					int n = 10;

					float x = fourier_square(t, 0, T_, n) * r_;
					float y = fourier_square(t, 0.5 * M_PI, T_, n) * r_;

					return (glm::vec3(x, y, 0) * r_);
				}
				/** radius */
				float	r_;
				/** period */
				float	T_;
				float	omega_;
		};
	}


	class Quat {
		public:
			virtual glm::quat	f(float) = 0;
	};
	class QuatConst: public Input::Quat {
		public:
			QuatConst(glm::quat q): q_(q) {}
			virtual glm::quat	f(float) { return q_; }

			glm::quat	q_;
	};
}

#endif

