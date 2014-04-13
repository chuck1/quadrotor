#include <cstdio>

#include <Print.hpp>

void print_arr(double* arr, int len) {
	for(int a = 0; a < len; a++) {
		printf("%f ", arr[a]);
	}
	printf("\n");
}

