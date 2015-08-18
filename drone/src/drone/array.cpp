
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

void			write(FILE* file, glm::vec3 const & v)
{
	fwrite(&v, sizeof(float), 3, file);
}
void			write(FILE* file, glm::vec4 const & v)
{
	fwrite(&v, sizeof(float), 4, file);
}
void			write(FILE* file, glm::quat const & v)
{
	assert(sizeof(glm::quat)==(4*sizeof(float)));
	fwrite(&v, sizeof(float), 4, file);
#if 0
	fwrite(&v.w, sizeof(float), 1, file);
	fwrite(&v.x, sizeof(float), 1, file);
	fwrite(&v.y, sizeof(float), 1, file);
	fwrite(&v.z, sizeof(float), 1, file);
#endif
}
void			write(FILE* file, float const & f)
{
	fwrite(&f, sizeof(float), 1, file);
}
void			read(FILE* file, glm::vec3 & v)
{
	fread(&v, sizeof(float), 3, file);
}
void			read(FILE* file, glm::vec4 & v)
{
	fread(&v, sizeof(float), 4, file);
}
void			read(FILE* file, glm::quat & v)
{
	fread(&v.w, sizeof(float), 1, file);
	fread(&v.x, sizeof(float), 1, file);
	fread(&v.y, sizeof(float), 1, file);
	fread(&v.z, sizeof(float), 1, file);
}
void			read(FILE* file, float & f)
{
	fread(&f, sizeof(float), 1, file);
}

