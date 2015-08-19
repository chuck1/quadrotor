#include <memory>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

#include <drone/Drone.hpp>
#include <drone/command/command.h>
#include <drone/command/Input.hpp>
#include <drone/Brain.hpp>
#include <drone/cl/Snap.hpp>

std::shared_ptr<Quadrotor> gdrone;
Command::X* gcmd;

void draw_shapes()
{
	glLoadIdentity();

	gluLookAt(
			0,10,10,
			0,0,0,
			0,1,0);

	/*glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
	glBegin(GL_TRIANGLES);
	glColor3f(1.f, 0.f, 0.f);
	glVertex3f(-0.6f, -0.4f, 0.f);
	glColor3f(0.f, 1.f, 0.f);
	glVertex3f(0.6f, -0.4f, 0.f);
	glColor3f(0.f, 0.f, 1.f);
	glVertex3f(0.f, 0.6f, 0.f);
	glEnd();
	*/

	glm::vec3 x = gdrone->x(gdrone->_M_i-1);

	glTranslatef(x.x,x.y,x.z);
	glutSolidCube(1);
}
void draw(GLFWwindow* window)
{
	float ratio;
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float) height;
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
	//glFrustum(-10,10,-10,10,-10,1000);
	gluPerspective(90,1,1,1000);
	glMatrixMode(GL_MODELVIEW);

	draw_shapes();	

	glfwSwapBuffers(window);
	glfwPollEvents();
}
static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	switch(action) {
	case GLFW_PRESS:
		switch(key) {
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			auto i = gcmd->get_input_is_const();
			assert(i);
			i->set(i->f(0) + glm::vec3(1,0,0));
		};
		break;
	}

}

GLFWwindow* window;



double t_0;

void		setup()
{
	gdrone.reset(new Quadrotor(1000));

	// command
	gcmd = new Command::X(gdrone.get(), new Input::Vec3::Const(glm::vec3(1,0,0)));

	gdrone->brain_->objs_.push_back(gcmd);
	
	// control parameters
	Jounce::X* x = dynamic_cast<Jounce::X*>(gdrone->brain_->cl_x_);
	assert(x);

	float poles[] = {-06.0,  -1.0,   0};
	int i[] = {0,0,1,1,2};
	x->set_poles(i, poles, 3);


	t_0 = glfwGetTime();
}
void		loop()
{
	double dt;
	double t_1;

	while(!glfwWindowShouldClose(window))
	{
		t_1 = glfwGetTime();
		dt = t_1 - t_0;
		t_0 = t_1;

		try {
			gdrone->step(dt);
		} catch(...) {
			printf("drone exception\n");
			break;
		}

		draw(window);


		// debug
		if(0) {
		printf("%12s%12s%12s\n", "dt", "t", "x");
		printf("%12f%12f%12f\n", dt,
				gdrone->t(gdrone->_M_i-1),
				gdrone->x(gdrone->_M_i-1).x);
		}

	}
}
int		main(int ac, char ** av)
{
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);
	window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);

	glutInit(&ac,av);

	setup();

	loop();

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}




