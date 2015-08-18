#ifndef __QUADROTOR_CONFIG__
#define __QUADROTOR_CONFIG__

class Quadrotor;

/** \brief Classes providing reference values to the controller
 *
 */
namespace Input {
	/** \brief Returning 3-element vector
	 */
	namespace Vec3 {
		class Base;
	}
}

namespace Command {
	class Base;
	class X;
}

/** \brief Control equation
 */
namespace CL {
	
	/** \brief Abstract classes indicating controller input
	 */
	namespace Input {

	}
	/** \brief Abstract classes indicating controller output
	 */
	namespace Output {

	}

}

#endif
