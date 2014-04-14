
#include <quadrotor/Input.hpp>

double fourier_square(double t, double phase, double L, int n) {
	double y = 0;
	int j;
	double t0 = phase / (2.0 * M_PI) * L;
	
	for(int i = 0; i < n; ++i) {
		j = 2 * i + 1;
		y += sin((double)j * M_PI * (t+t0) / L) / (double)j;
	}
	y *= 4.0 / M_PI;
	return y;
}

