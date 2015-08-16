
#include <cstdio>

#include <math/vec3.hpp>
#include <math/vec4.hpp>
#include <math/quat.hpp>

void write(FILE* file, math::vec3 const & v) {
	v.write(file);
}
void write(FILE* file, math::vec4 const & v) {
	fwrite(&v, sizeof(double), 4, file);
}
void write(FILE* file, math::quat const & v) {
	fwrite(&v.w, sizeof(double), 1, file);
	fwrite(&v.x, sizeof(double), 1, file);
	fwrite(&v.y, sizeof(double), 1, file);
	fwrite(&v.z, sizeof(double), 1, file);
}
void write(FILE* file, double const & f) {
	fwrite(&f, sizeof(double), 1, file);
}

void read(FILE* file, math::vec3 & v) {
	v.read(file);
}
void read(FILE* file, math::vec4 & v) {
	fread(&v, sizeof(double), 4, file);
}
void read(FILE* file, math::quat & v) {
	fread(&v.w, sizeof(double), 1, file);
	fread(&v.x, sizeof(double), 1, file);
	fread(&v.y, sizeof(double), 1, file);
	fread(&v.z, sizeof(double), 1, file);
}
void read(FILE* file, double & f) {
	fread(&f, sizeof(double), 1, file);
}

