#ifndef __EXCEPT__
#define __EXCEPT__

#include <gal/except.h>

class StopCond: public gal::except {
	public:
		StopCond(int ti): ti_(ti) {}
		virtual const char * what() const throw ()  = 0;
		//	return "stop condition";
		//}
		int ti_;
};

class EmptyQueue: public StopCond {
	public:
		EmptyQueue(int ti): StopCond(ti) {}
		virtual const char * what() const  throw () {
			return "empty queue";
		}
};

class OmegaHigh: public StopCond {
	public:
		OmegaHigh(int ti): StopCond(ti) {}
		virtual const char * what() const  throw () {
			return "omega high";
		}
};

class InvalidOp: public StopCond {
	public:
		InvalidOp(int ti): StopCond(ti) {}
		virtual const char * what() const  throw () {
			return "invalid operation";
		}
};


#endif


