#ifndef __QUADROTOR_INPUT__
#define __QUADROTOR_INPUT__

namespace Input {
	class Vec3 {
		public:
			virtual math::vec3	f(double) = 0;
	};
	class Vec3Const: public Input::Vec3 {
		public:
			Vec3Const(math::vec3 v): v_(v) {}
			virtual math::vec3	f(double) { return v_; }

			math::vec3	v_;
	};
	class Quat {
		public:
			virtual math::quat	f(double) = 0;
	};
	class QuatConst: public Input::Quat {
		public:
			QuatConst(math::quat q): q_(q) {}
			virtual math::quat	f(double) { return q_; }

			math::quat	q_;
	};
}

#endif

