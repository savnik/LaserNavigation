
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef WIN32
#include <Windows.h>
#endif

#ifndef WIN32
#include <sys/time.h>
#include <string.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


#include "ac3d.h"


#define WINDOW_WIDTH    768
#define WINDOW_HEIGHT   576

#define JOYSTICKUP 101
#define JOYSTICKDOWN 103
#define JOYSTICKLEFT 100
#define JOYSTICKRIGHT 102

int window_width = WINDOW_WIDTH;
int window_height = WINDOW_HEIGHT;

int keypressed = 0, modifier = 0;
int display_list[5];
double zoom = 0.0, yrot = 45.0, xrot = 0.0;
double tower_rot = 0.0, arm_rot = 45.0, ext_arm = 0.0, gun_rot = 0.0, head_rot = 0.0;


void set_projection(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)w/(float)h, 0.1, 10000.0);
}

void init_gfx(void)
{
    ac_prepare_render();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    set_projection(WINDOW_WIDTH,WINDOW_HEIGHT);
}


#ifdef WIN32
#include <time.h>
#endif

struct timeval tv;
int ts, tu;
int ets, etu;

#ifdef WIN32
void gettimeofday( struct timeval *tv, void *dummy )
{
	LARGE_INTEGER perfCount;
	LARGE_INTEGER perfFreq;

	double freq, count, time;
	tv->tv_sec = 0;
	tv->tv_usec= 0;

	// Get the frequency
	if( !QueryPerformanceFrequency( &perfFreq ) )
		return;

	// Get the current count.
	if( !QueryPerformanceCounter( &perfCount ) )
		return;

	freq  = (double) perfFreq.LowPart;
	count = (double) perfCount.LowPart;
	freq += (double) perfFreq.HighPart  *  4294967296.0;
	count+= (double) perfCount.HighPart *  4294967296.0;

	time = count / freq;

	tv->tv_sec = (int) time;
	tv->tv_usec= (int) ((time - (double)tv->tv_sec) * 1000000.0);

}
#endif // WIN32



void timer_start()
{
    gettimeofday(&tv, NULL);
    ts = tv.tv_sec;
    tu = tv.tv_usec;
}

float timer_stop()
{
    int s, u;
    
    gettimeofday(&tv, NULL);
    ets = tv.tv_sec;
    etu = tv.tv_usec;
    
    s = ets-ts;
    u = etu-tu;
    return(s+u/1.0E+6);
}

void update_robot(float frametime) {

  // Update robot positions based on keyboard input
  switch(modifier) {
    // Button 1 on the joystick
  case GLUT_ACTIVE_SHIFT:
    switch(keypressed) {
    case JOYSTICKUP:
      gun_rot += 20 * frametime;
      if (gun_rot > 0)
	gun_rot = 0;
      break;
    case JOYSTICKDOWN:
      gun_rot -= 20 * frametime;
      if (gun_rot < -255)
	gun_rot = -255;
      break;

    case JOYSTICKLEFT:
      ext_arm += .1 * frametime;
      if (ext_arm > 1.35)
	ext_arm = 1.35;
      break;

    case JOYSTICKRIGHT:
      ext_arm -= .1 * frametime;
      if (ext_arm < 0)
	ext_arm = 0;
      break;
    default:

      break;
    }
    break;
  case GLUT_ACTIVE_CTRL:
    // Button 2 on the joystick
    switch(keypressed) {
    case JOYSTICKUP:

      break;
    case JOYSTICKDOWN:

      break;

    case JOYSTICKLEFT:
      head_rot += 40 * frametime;
      break;

    case JOYSTICKRIGHT:
      head_rot -= 40 * frametime;
      break;
    default:

      break;
    }
    break;
  case GLUT_ACTIVE_ALT:
    // None of the joystick buttons, used to rotate the entire scene
    switch(keypressed) {
    case JOYSTICKUP:
      xrot -= 50 * frametime;
      break;
    case JOYSTICKDOWN:
      xrot += 50 * frametime;
      break;

    case JOYSTICKLEFT:
      yrot += 50 * frametime;
      break;

    case JOYSTICKRIGHT:
      yrot -= 50 * frametime;
      break;
    default:

      break;
    }

    break;
  default:
    switch(keypressed) {
    case JOYSTICKUP:
      arm_rot += 20 * frametime;
      if (arm_rot > 45)
	arm_rot = 45;
      break;
    case JOYSTICKDOWN:
      arm_rot -= 20 * frametime;
      if (arm_rot < -60)
	arm_rot = -60;
      break;

    case JOYSTICKLEFT:
      tower_rot += 20 * frametime;
      if (tower_rot > 200)
	tower_rot = 200;
      break;

    case JOYSTICKRIGHT:
      tower_rot -= 20 * frametime;
      if (tower_rot < -200)
	tower_rot = -200;
      break;

    case 113: case 81: case 27:
      // q, Q and ESC exits
      exit(0);
      break;

    case 122: case 90: // z and Z
      zoom += 2.5 * frametime;
      break;

    case 120: case 88: // x and X
      zoom -= 2.5 * frametime;
      break;

    default:

      break;
    }
    break;
  }
}

void drawscene(void) {
  static float frametime;
  static float tottime = 0.0;
  static int framec = 0;

  GLfloat light_position[] = { 0.0, 1000.0, 1000.0, 0.0 };

  timer_start();


  set_projection(window_width, window_height);

  gluLookAt(0, 1.5, 4.0, 0, 0, 0, 0, 1, 0);

  glClearColor(0,0,0,0);
  glClearColor(0.1,0.1,0.4,0.0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  // "zoom" in or out and rotate entire scene
  glTranslatef(0.0, 0.0, zoom);
  glRotatef(xrot, 1.0, 0.0, 0.0);
  glRotatef(yrot, 0.0, 1.0, 0.0);

  update_robot(frametime);

  glPushMatrix();
  glTranslatef(-.25, 0.1, 0.0);
  glScalef(0.25, 0.25, 0.25);
  glutSolidTeapot(.2);
  glPopMatrix();

  // Draw the chasis at origo
  glCallList(display_list[0]);

  // Draw the tower 0.4m over the chasis, rotated about the Y-axis
  glRotatef(tower_rot, 0.0 , 1.0 , 0.0);
  glTranslatef(0.0, 0.6, 0.0);
  glCallList(display_list[1]);

  // Draw the arm 0.6m over the tower and rotated about the Z-axis
  glTranslatef(0.0, .6, 0.0);
  glRotatef(arm_rot, 0.0, 0.0, 1.0);
  glCallList(display_list[2]);

  // Draw the extended arm translated along the -X-axis
  glTranslatef(-ext_arm, 0.0, 0.0);
  glCallList(display_list[3]);

  // Draw the gun rotated, Z-axis first, then X-axis
  glTranslatef(-1.225, 0.0, 0.075);
  glRotatef(gun_rot, 0.0, 0.0, 1.0);
  glRotatef(head_rot, 1.0 , 0.0 , 0.0);
  glCallList(display_list[4]);

  glFlush();
  glFinish();
  glutSwapBuffers();

  frametime = timer_stop();
  tottime += frametime;
  framec++;
  if (tottime > 5.0)
    {
      printf("approx frames per sec: %f\n", 1.0/(tottime/framec));
      tottime = 0;
      framec = 0;
    }

}

void reshape_gfx(int w, int h)
{
    window_width = w; window_height = h;
    glViewport(0, 0, w, h);
    set_projection(w,h);
}

void keyboard(unsigned char key, int x, int y) {
  if (keypressed) {
    keypressed = 0;
    modifier = 0;
    return;
  }
  printf("Normal key: %d\n",key);
  keypressed = key;
  modifier = glutGetModifiers();
}

void keyboardUp(unsigned char key, int x, int y) {
  keypressed = 0;
  modifier = 0;
}

void special(int key, int x, int y) {
  if (keypressed) {
    keypressed = 0;
    modifier = 0;
    return;
  }
  //  printf("Special key: %d\n",key);
  keypressed = key;
  modifier = glutGetModifiers();
}

void specialUp(int key, int x, int y) {
  keypressed = 0;
  modifier = 0;
}

int main(int argc, char *argv[])
{
    ACObject *ob[5];
    //    static char *acFileName="3d/alto_sketch.ac";

    glutInit(&argc, argv);

    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

    glutCreateWindow(argv[0]);

    glutReshapeFunc(reshape_gfx);
    glutDisplayFunc(drawscene);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(special);
    glutSpecialUpFunc(specialUp);

    glutIdleFunc(drawscene);

    init_gfx();
    /*
    if ( argc > 1 ) acFileName = argv[argc-1];
    printf("acFileName: %s\n",acFileName);
    */
    ob[0] = ac_load_ac3d("3d/alto_chasis.ac");
    ob[1] = ac_load_ac3d("3d/alto_tower.ac");
    ob[2] = ac_load_ac3d("3d/alto_arm.ac");
    ob[3] = ac_load_ac3d("3d/alto_extarm.ac");
    ob[4] = ac_load_ac3d("3d/alto_gun.ac");

    display_list[0] = ac_display_list_render_object(ob[0]);
    display_list[1] = ac_display_list_render_object(ob[1]);
    display_list[2] = ac_display_list_render_object(ob[2]);
    display_list[3] = ac_display_list_render_object(ob[3]);
    display_list[4] = ac_display_list_render_object(ob[4]);

    glutMainLoop();

    return 0;
}
