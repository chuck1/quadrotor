#ifndef __SEARCH__
#define __SEARCH__

class Quadrotor;

class search {
	public:
		search(int*, float*, int);
		void		reset();
		void		step();
		void		exec(int);
		bool		test();

		Quadrotor*	r_;
		int*		i_;
		float*		v_;
		int		nv_;
		float		ts_;
		int		current_;
		int		n_;
		int		count_;
};

#endif

