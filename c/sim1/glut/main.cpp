
//glut_example.c
// Stanford University, CS248, Fall 2000
//
// Demonstrates basic use of GLUT toolkit for CS248 video game assignment.
// More GLUT details at http://reality.sgi.com/mjk_asd/spec3/spec3.html
// Here you'll find examples of initialization, basic viewing transformations,
// mouse and keyboard callbacks, menus, some rendering primitives, lighting,
// double buffering, Z buffering, and texturing.
//
// Matt Ginzton -- magi@cs.stanford.edu

#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <math/transform.h>
#include <math/mat44.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/color.h>

#include <quadrotor/array.h>

#define VIEWING_DISTANCE_MIN  3.0
#define TEXTURE_ID_CUBE 1

typedef int BOOL;
#define TRUE 1
#define FALSE 0

// globals
float g_view_yaw = 0;
float g_view_pitch = 0;
int g_n = 0;

int ti = 0;

Array<math::vec3> g_x;
Array<math::vec3> g_v;
Array<math::vec3> g_j;
Array<math::vec3> g_s;
Array<math::quat> g_q;
Array<math::vec3> g_o;

int g_cam_mode = 0;

static BOOL g_bLightingEnabled = TRUE;
static BOOL g_bFillPolygons = TRUE;
static BOOL g_bButton1Down = FALSE;
static GLfloat g_fTeapotAngle = 0.0;
static GLfloat g_fTeapotAngle2 = 0.0;
static GLfloat g_nearPlane = 1;
static GLfloat g_farPlane = 1000;
static int g_Width = 600;                          // Initial window width
static int g_Height = 600;                         // Initial window height
static float g_lightPos[4] = { 0, 10, 10, 1 };  // Position of light
#ifdef _WIN32
static DWORD last_idle_time;
#else
static struct timeval last_idle_time;
#endif

float g_view_x = 0.0;//0.0;
float g_view_y = 0.0;
float g_view_z = 0.0;//7.5;
float g_view_dist = 25.0;


enum {
	MENU_LIGHTING = 1,
	MENU_POLYMODE,
	MENU_CAMERA,
	MENU_EXIT
};
enum {
	CAM_STATIONARY = 0,
	CAM_FOLLOW,
	CAM_COUNT
};


void RenderObjects(void)
{
	//float colorBronzeDiff[4] = { 0.8, 0.6, 0.0, 1.0 };
	//float colorBronzeSpec[4] = { 1.0, 1.0, 0.4, 1.0 };

	math::color arm_color[] = {
		math::red,
		math::yellow,
		math::blue,
		math::green};
	
	
	math::transform t(g_x[ti], g_q[ti].getConjugate());
	
	math::mat44 m(t);

	// arms
	float xa[] = {1,-1,0, 0};
	float ya[] = {0, 0,1,-1};
	float L = 1.0;
	
	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	{
		glTranslatef(0.0,0.0,-2.0);
		glScalef(10.0,10.1,1.0);
		glutSolidCube(1.0);
	}
	glPopMatrix();
	
	glPushMatrix();
	{
		glTranslatef(-g_view_x, -g_view_y, 0);

		// Main object (cube) ... transform to its coordinates, and render
		glMultMatrixf(m);

		glMaterialfv(GL_FRONT, GL_DIFFUSE, math::cyan);
		glMaterialfv(GL_FRONT, GL_SPECULAR, math::white);
		glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

		glutSolidSphere(0.5,20,20);
		
		for(int i = 0;i< 4; i++) {
			// Child object (teapot) ... relative transform, and render
			glPushMatrix();
			{
				glTranslatef(xa[i]*L, ya[i]*L, 0);
				//glRotatef(g_fTeapotAngle2, 1, 1, 0);
				
				glMaterialfv(GL_FRONT, GL_DIFFUSE, arm_color[i]);
				glMaterialfv(GL_FRONT, GL_SPECULAR, math::white);
				glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
				//glColor4fv(arm_color[i]);

				glutSolidTorus(0.1,0.2,20,20);

			}
			glPopMatrix(); 
		}
	}
	glPopMatrix();

	ti++;
	if(ti == g_n) ti = 0;
}


void display(void)
{
	// Clear frame buffer and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set up viewing transformation, looking down -Z axis
	glLoadIdentity();
	
	gluLookAt(
			0.0, g_view_dist, g_view_z,
			0.0, 0.0, g_view_z,
			0.0, 0.0, 1);

	glRotatef(g_view_yaw/M_PI*180.0,   0, 0, 1);
	
	glRotatef(-g_view_pitch/M_PI*180.0, cos(g_view_yaw), -sin(g_view_yaw), 0);
	
	glPushMatrix();
	{
		// Set up the stationary light
		glLightfv(GL_LIGHT0, GL_POSITION, g_lightPos);

		// Render the scene
		RenderObjects();
	}
	glPopMatrix();

	// Make sure changes appear onscreen
	glutSwapBuffers();
}

void reshape(GLint width, GLint height)
{
	g_Width = width;
	g_Height = height;

	glViewport(0, 0, g_Width, g_Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (float)g_Width / g_Height, g_nearPlane, g_farPlane);
	glMatrixMode(GL_MODELVIEW);
}

void InitGraphics(void)
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// Create texture for cube; load marble texture from file and bind it

}

int g_mx = 0;
int g_my = 0;

void MouseButton(int button, int state, int x, int y)
{
	// Respond to mouse button presses.
	// If button1 pressed, mark this state so we know in motion function.

	//printf("button\n");

	if (button == GLUT_LEFT_BUTTON)
	{
		g_bButton1Down = (state == GLUT_DOWN) ? TRUE : FALSE;
	
		if(state == GLUT_DOWN) {
			g_mx = x;
			g_my = y;
		}
	}
}


void MouseMotion(int x, int y)
{
	// If button1 pressed, zoom in/out if mouse is moved up/down.

	if (g_bButton1Down)
	{
		//g_fViewDistance = (y - g_yClick) / 3.0;
		//if (g_fViewDistance < VIEWING_DISTANCE_MIN)
		//	g_fViewDistance = VIEWING_DISTANCE_MIN;
		//glutPostRedisplay();
		
		int dx = x - g_mx;
		int dy = y - g_my;
		
		g_view_yaw += (float)dx * 3.14 / 300.0;
		g_view_pitch += (float)dy * 3.14 / 300.0;
		
		g_mx = x;
		g_my = y;

		//printf("%i %i\n", g_mx, x);
	}
}

void AnimateScene(void)
{
	float dt;

	// Figure out time elapsed since last call to idle function
	struct timeval time_now;
	gettimeofday(&time_now, NULL);
	dt = (float)(time_now.tv_sec  - last_idle_time.tv_sec) +
		1.0e-6*(time_now.tv_usec - last_idle_time.tv_usec);

	// Animate the teapot by updating its angles
	g_fTeapotAngle += dt * 30.0;
	g_fTeapotAngle2 += dt * 100.0;

	// Save time_now for next time
	last_idle_time = time_now;

	// Force redraw
	glutPostRedisplay();
}

void SelectFromMenu(int idCommand)
{
	switch (idCommand)
	{
		case MENU_LIGHTING:
			g_bLightingEnabled = !g_bLightingEnabled;
			if (g_bLightingEnabled)
				glEnable(GL_LIGHTING);
			else
				glDisable(GL_LIGHTING);
			break;
		case MENU_POLYMODE:
			g_bFillPolygons = !g_bFillPolygons;
			glPolygonMode (GL_FRONT_AND_BACK, g_bFillPolygons ? GL_FILL : GL_LINE);
			break;      
		case MENU_CAMERA:
			g_cam_mode++;
			if (g_cam_mode == 2) g_cam_mode = 0;
		case MENU_EXIT:
			exit (0);
			break;
	}

	// Almost any menu selection requires a redraw
	glutPostRedisplay();
}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:             // ESCAPE key
			exit (0);
			break;

		case 'l':
			SelectFromMenu(MENU_LIGHTING);
			break;

		case 'p':
			SelectFromMenu(MENU_POLYMODE);
			break;
		case 'c':
			SelectFromMenu(MENU_CAMERA);

	}
}

int BuildPopupMenu (void)
{
	int menu;

	menu = glutCreateMenu (SelectFromMenu);
	glutAddMenuEntry ("Toggle lighting\tl", MENU_LIGHTING);
	glutAddMenuEntry ("Toggle polygon fill\tp", MENU_POLYMODE);
	glutAddMenuEntry ("Cycle camera mode\tp", MENU_CAMERA);
	glutAddMenuEntry ("Exit demo\tEsc", MENU_EXIT);

	return menu;
}

int file_size(FILE* fp) {
	fseek(fp, 0L, SEEK_END);
	int sz = ftell(fp);
	
	//You can then seek back to the beginning:
	
	fseek(fp, 0L, SEEK_SET);

	return sz;
}
void load() {
	FILE* file = fopen("../data/telem.txt", "r");
	
	if(file == NULL) {
		printf("file not found\n");
		abort();
	}
	
	int types = (5*3 + 1*4) * 8;

	g_n = file_size(file) / types;
		
	printf("time %i\n", g_n);
	
	g_x.alloc(g_n);
	g_v.alloc(g_n);
	g_j.alloc(g_n);
	g_s.alloc(g_n);
	g_q.alloc(g_n);
	g_o.alloc(g_n);
	
	g_x.read(file, g_n);
	g_v.read(file, g_n);
	g_j.read(file, g_n);
	g_s.read(file, g_n);
	g_q.read(file, g_n);
	g_o.read(file, g_n);

}
int main(int argc, char** argv)
{
	ti = 0;

	load();

	// GLUT Window Initialization:
	glutInit (&argc, argv);
	glutInitWindowSize (g_Width, g_Height);
	glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow ("CS248 GLUT example");

	// Initialize OpenGL graphics state
	InitGraphics();

	// Register callbacks:
	glutDisplayFunc (display);
	glutReshapeFunc (reshape);
	glutKeyboardFunc (Keyboard);
	glutMouseFunc (MouseButton);
	glutMotionFunc (MouseMotion);
	glutIdleFunc (AnimateScene);

	// Create our popup menu
	BuildPopupMenu ();
	glutAttachMenu (GLUT_RIGHT_BUTTON);

	// Get the initial time, for use by animation
	gettimeofday (&last_idle_time, NULL);

	// Turn the flow of control over to GLUT
	glutMainLoop ();
	return 0;
}




