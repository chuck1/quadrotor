
#include <drone/command/Input.hpp>

float fourier_square(float t, float phase, float L, int n) {
	float y = 0;
	int j;
	float t0 = phase / (2.0 * M_PI) * L;
	
	for(int i = 0; i < n; ++i) {
		j = 2 * i + 1;
		y += sin((float)j * M_PI * (t+t0) / L) / (float)j;
	}
	y *= 4.0 / M_PI;
	return y;
}

