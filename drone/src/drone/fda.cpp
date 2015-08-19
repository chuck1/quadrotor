
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <drone/util/print.hpp>
#include <drone/fda.h>

void zero(glm::vec3& a)
{
	a = glm::vec3(0);
}
void zero(double& a)
{
	a = 0.0;
}
bool sane(glm::vec3 const & a)
{
	return true;//a.IsSane();
}
bool sane(double const & a)
{
	if(isnan(a)) return false;
	if(isinf(a)) return false;
	return true;
}
int		drone::print(glm::quat const & a)
{
	printf("%16e%16e%16e%16e\n", a.w, a.x, a.y, a.z);
	return 0;
}
int		drone::print(glm::vec3 const & a)
{
	printf("%16e%16e%16e\n", a.x, a.y, a.z);
	return 0;
}
int		drone::print(glm::vec4 const & a)
{
	printf("%16e%16e%16e%16e\n", a.w, a.x, a.y, a.z);
	return 0;
}
int		drone::print(glm::mat4 const & a)
{
	printf("%16e%16e%16e%16e\n", a[0][0], a[0][1], a[0][2], a[0][3]);
	printf("%16e%16e%16e%16e\n", a[1][0], a[1][1], a[1][2], a[1][3]);
	printf("%16e%16e%16e%16e\n", a[2][0], a[2][1], a[2][2], a[2][3]);
	printf("%16e%16e%16e%16e\n", a[3][0], a[3][1], a[3][2], a[3][3]);
	return 0;
}
int		drone::print(double const & a)
{
	printf("%lf\n",a);
	return 0;
}
glm::quat diff(glm::quat const & a, glm::quat const & b)
{
	return (a * glm::conjugate(b));
}
glm::vec3		get_omega(glm::quat q, float h)
{
	q = glm::normalize(q);

	float theta = 2.f * acos(q.w);

	glm::vec3 a = glm::vec3(q.x,q.y,q.z) / (float)sin(theta/2.f);

	printf("get omega\n");
	printf("h = %f\n", h);
	printf("theta = %f\n", theta);
	printf("q =\n");
	drone::print(q);
	printf("a =\n");
	drone::print(a);

	return theta / h * a;
}
void			forward_quavec(
		Array<glm::quat> q,
		Array<glm::vec3> qd, double h, int ti)
{
	//q[ti].print();
	//q[ti-1].print();
	
	glm::quat r = diff(q[ti], q[ti-1]);
	qd[ti] = get_omega(r, h);
}




