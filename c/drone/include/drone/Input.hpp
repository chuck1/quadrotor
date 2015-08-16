#ifndef __QUADROTOR_INPUT__
#define __QUADROTOR_INPUT__

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

double fourier_square(double t, double phase, double L, int n);

namespace Input {
	namespace Vec3 {
		class Base {
			public:
				virtual glm::vec3	f(double) = 0;
		};
		class Const: public Input::Vec3::Base {
			public:
				Const(glm::vec3 v): v_(v) {}
				virtual glm::vec3	f(double) { return v_; }

				glm::vec3	v_;
		};
		class Circle: public Input::Vec3::Base {
			public:
				Circle(double r, double T): r_(r), T_(T) {
					omega_ = 2.0 * M_PI / T_;
				}
				virtual glm::vec3	f(double t) {

					double x = fourier_square(t, 0, T_, 4) * r_;
					double y = fourier_square(t, 0.5 * M_PI, T_, 4) * r_;

					//double x = sin(omega_ * t);
					//double y = cos(omega_ * t);
					return (glm::vec3(x, y, 0) * r_);
				}

				float	r_;
				double	T_;
				double	omega_;
		};
	}


	class Quat {
		public:
			virtual glm::quat	f(double) = 0;
	};
	class QuatConst: public Input::Quat {
		public:
			QuatConst(glm::quat q): q_(q) {}
			virtual glm::quat	f(double) { return q_; }

			glm::quat	q_;
	};
}

#endif

