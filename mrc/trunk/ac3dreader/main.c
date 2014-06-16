
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef WIN32
#include <Windows.h>
#endif

#ifndef WIN32
#include <sys/time.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


#include "ac3d.h"


#define WINDOW_WIDTH    768
#define WINDOW_HEIGHT   576

int window_width = WINDOW_WIDTH;
int window_height = WINDOW_HEIGHT;

int display_list;


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

void drawscene(void)
{
    static float rot = 0.0;
    float frametime;
    static float tottime = 0.0;
    static int framec = 0;


    GLfloat light_position[] = { -1000.0, 1000.0, 1000.0, 0.0 };

    timer_start();


    set_projection(window_width, window_height);

    gluLookAt(0, 2.8, 4, 0, 0, 0, 0, 1, 0);

    glClearColor(0,0,0,0);
    glClearColor(0.5,0.5,0.5,0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glRotatef(rot, 0, 1, 0);

    glCallList(display_list);
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

    rot = rot + 20 * frametime;
}

void reshape_gfx(int w, int h)
{
    window_width = w; window_height = h;
    glViewport(0, 0, w, h);
    set_projection(w,h);
}


int main(int argc, char *argv[])
{
    ACObject *ob;
    static char *acFileName="test.ac";

    glutInit(&argc, argv);

    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

    glutCreateWindow(argv[0]);

    glutReshapeFunc(reshape_gfx);
    glutDisplayFunc(drawscene);

    glutIdleFunc(drawscene);

    init_gfx();

    if ( argc > 1 ) acFileName = argv[argc-1];
    printf("acFileName: %s\n",acFileName);
    ob = ac_load_ac3d(acFileName);

    display_list = ac_display_list_render_object(ob);

    glutMainLoop();

    return 0;
}
