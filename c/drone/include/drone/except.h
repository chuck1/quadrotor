#ifndef __EXCEPT__
#define __EXCEPT__

#include <exception>

class StopCond: public std::exception {
	public:
		virtual const char * what() const throw ()  = 0;
		//	return "stop condition";
		//}
};

class EmptyQueue: public StopCond {
	public:
		virtual const char * what() const  throw () {
			return "empty queue";
		}
};

class OmegaHigh: public StopCond {
	public:
		virtual const char * what() const  throw () {
			return "omega high";
		}
};

class InvalidOp: public StopCond {
	public:
		virtual const char * what() const  throw () {
			return "invalid operation";
		}
};
class Inf: public StopCond {
	public:
		virtual const char * what() const  throw () {
			return "inf";
		}
};	

#endif


