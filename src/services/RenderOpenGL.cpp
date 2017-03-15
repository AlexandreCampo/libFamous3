/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2017 Alexandre Campo                                 */
/*                                                                            */
/*    This file is part of FaMouS  (a fast, modular and simple simulator).    */
/*                                                                            */
/*    FaMouS is free software: you can redistribute it and/or modify          */
/*    it under the terms of the GNU General Public License as published by    */
/*    the Free Software Foundation, either version 3 of the License, or       */
/*    (at your option) any later version.                                     */
/*                                                                            */
/*    FaMouS is distributed in the hope that it will be useful,               */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*    GNU General Public License for more details.                            */
/*                                                                            */
/*    You should have received a copy of the GNU General Public License       */
/*    along with FaMouS.  If not, see <http://www.gnu.org/licenses/>.         */
/*----------------------------------------------------------------------------*/

// code of this render is adapted from drawstuff lib of ODE. An ugly tool, but quickly set up. 

#include "RenderOpenGLInterface.h"
#include "Simulator.h"


#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <string>
#include <fstream>

#include <sys/time.h>

#include "RenderOpenGL.h"


#define ESCAPE 27
#define BACKSPACE 8

// Static callbacks
// ================

static RenderOpenGL* RenderOpenGLInstance = 0;


static void glutKeyboardCallback(unsigned char key, int x, int y)
{
    RenderOpenGLInstance->keyboardCallback(key,x,y);
}

static void glutKeyboardUpCallback(unsigned char key, int x, int y)
{
    RenderOpenGLInstance->keyboardUpCallback(key,x,y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y)
{
    RenderOpenGLInstance->specialKeyboard(key,x,y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y)
{
    RenderOpenGLInstance->specialKeyboardUp(key,x,y);
}


static void glutReshapeCallback(int w, int h)
{
    RenderOpenGLInstance->reshape(w,h);
}

static void glutIdleCallback()
{
    RenderOpenGLInstance->idle();
}

static void glutMouseCallback(int button, int state, int x, int y)
{
    RenderOpenGLInstance->mouse(button,state,x,y);
}


static void glutMotionCallback(int x,int y)
{
    RenderOpenGLInstance->mouseMotion(x,y);
}


static void glutDisplayCallback(void)
{
    RenderOpenGLInstance->displayCallback();
}

// =============================
// RenderOpenGL Methods
// =============================


RenderOpenGL::RenderOpenGL(int argc, char** argv)
{
    RenderOpenGLInstance = this;

    this->argc = argc;
    this->argv = argv;

    ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
    ground_ofsx = 0.5;		// offset of ground texture
    ground_ofsy = 0.5;
    sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
    sky_height = 1.0f;		// sky height above viewpoint
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 0;
    sphere_quality = 1;
    capped_cylinder_quality = 3;
    current_state = 0;
    use_textures=0;		// 1 if textures to be drawn
    use_shadows=0;		// 1 if shadows to be drawn

    view_xyz[0] = 0;	// position x,y,z
    view_xyz[1] = 0;	// position x,y,z
    view_xyz[2] = 20.0;	// position x,y,z
    view_hpr[0] = 0;	// heading, pitch, roll (degrees)
    view_hpr[1] = -90.0;	// heading, pitch, roll (degrees)
    view_hpr[2] = 0;	// heading, pitch, roll (degrees)
    view_up[0] = 0;	// up vector
    view_up[1] = 0;	// up vector
    view_up[2] = 1;	// up vector

    aperture = 45.0 * M_PI / 180.0;
    vnear = 1.0;
    vfar = 1000.0;

    // look at 0,0,0
    view_lookat[0] = 0.0;
    view_lookat[1] = 0.0;
    view_lookat[2] = 0.0;
    
    // camera is at distance, alpha, beta
    view_dab[0] = 10.0;
    view_dab[1] = 0.0;
    view_dab[2] = 45.0;

    view_mode = 1;

    mouseButton = 0;
    mouseLastX = 0;
    mouseLastY = 0;
    mouseX = 0;
    mouseY = 0;

    zoomCoefficient = 0.2;

    simulationElapsedTicks = 0.0;
    refreshDelay = (long int) (1.0 / 24.0 * 1000.0 * 1000.0);
    gettimeofday(&lastTv, NULL);   
    recalculateTimings();

    paused = false;

    // create window, init opengl  etc...
  /* Initialize GLUT state - glut will take any command line arguments that pertain to it or 
     X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html */  
  glutInit(&argc, argv);  

  /* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffered for automatic clipping */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* get a 640 x 480 window */
  glutInitWindowSize(800, 600);  

  /* the window starts at the upper left corner of the screen */
  glutInitWindowPosition(0, 0);  

  /* Open a window */  
  window = glutCreateWindow("Simulator");  

  // callbacks
  glutKeyboardFunc (glutKeyboardCallback);
  glutKeyboardUpFunc (glutKeyboardUpCallback);
  glutSpecialFunc (glutSpecialKeyboardCallback);
  glutSpecialUpFunc (glutSpecialKeyboardUpCallback);  
  glutReshapeFunc (glutReshapeCallback);
  glutIdleFunc (glutIdleCallback);
  glutMouseFunc (glutMouseCallback);
  glutPassiveMotionFunc (glutMotionCallback);
  glutMotionFunc (glutMotionCallback);
  glutDisplayFunc (glutDisplayCallback );  

  glutIdleCallback();

  /* Go fullscreen.  This is as soon as possible. */
  // glutFullScreen();
  fullscreen = false;

  /* Initialize our window. */
  initGL(800, 600);


  // display some basic information for end user
  std::cout << "OpenGL Render is active, the following shortcuts are available :" << std::endl;
  std::cout << "----------------------------------------------------------------" << std::endl;
//  std::cout << "The following shortcuts are available :" << std::endl;
  std::cout << "      escape : quit the program" << std::endl;
  std::cout << "       space : toggle pause" << std::endl;
  std::cout << "           - : slow down simulation" << std::endl;
  std::cout << "      + or = : accelerate simulation" << std::endl;
  std::cout << "   backspace : real time speed" << std::endl;
  std::cout << "           t : display current time in simulation" << std::endl;
  std::cout << "           r : reset simulation" << std::endl;
  std::cout << "  " << std::endl;
}

RenderOpenGL::~RenderOpenGL ()
{

}

// if timestep is changed, we have to call that function
void RenderOpenGL::recalculateTimings ()
{
    simulationTicksDelay = simulator->timestep / (1.0 / 24.0);
}

void RenderOpenGL::step ()
{
}

void RenderOpenGL::drawScene ()
{
    dsDrawFrame();

    // draw the objects
    std::list<RenderOpenGLInterface*>::iterator it;
    for (it = objects.begin(); it != objects.end(); it++)
    {
	(*it)->draw(this);
    }

    glLoadIdentity();
    glutSwapBuffers();
}

/* A general OpenGL initialization function.  sets all of the initial parameters. */
// We call this right after our OpenGL window is created.
void RenderOpenGL::initGL(int width, int height)
{
    this->width = width;
    this->height = height;
    
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
    glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
    glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading
    
    setPerspective (aperture, vnear, vfar);
}


/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void RenderOpenGL::reshape(int width, int height)
{
  if (height==0)				// Prevent A Divide By Zero If The Window Is Too Small
    height=1;

  this->width = width;
  this->height = height;

  glViewport(0, 0, width, height);		// reset The Current Viewport And Perspective Transformation

  setPerspective (aperture, vnear, vfar);
}

void RenderOpenGL::setPerspective (float aperture, float near, float far)
{
    this->aperture = aperture;
    this->vnear = near;
    this->vfar = far;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    gluPerspective(aperture,(GLfloat)width/(GLfloat)height, near, far);
    glMatrixMode(GL_MODELVIEW);
}


void RenderOpenGL::setPaused (bool p)
{
    paused = p;
}


/* The function called whenever a key is pressed. */
void RenderOpenGL::keyboardCallback(unsigned char key, int x, int y) 
{
    /* avoid thrashing this call */
    usleep(100);

    if (key == ' ') 
    { 
	if (paused) 
	{
	    paused = false;
	}
	else
	{
	    paused = true;
	}
	std::cout << "Pause toggled ... " << std::endl;
    }

    /* If escape is pressed, kill everything. */
    if (key == ESCAPE) 
    { 
	glutDestroyWindow(window); 
	delete simulator;
	exit(0);                   
    }

    // increase simulation speed
    if (key == '+' || key == '=')
    {
	// that reduce delay
	simulationTicksDelay /= 1.5;
//	if (simulationTicksDelay < 1.0) simulationTicksDelay = 1.0;
	std::cout << "Accelerate and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << std::endl;
    }

    if (key == '-')
    {
	// that reduce delay
	simulationTicksDelay *= 1.5;
	std::cout << "Slow down and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << std::endl;
    }

    if (key == BACKSPACE)
    {
	simulationTicksDelay = 1.0;
	std::cout << "reset speed and run realtime" << std::endl;
    }

    if (key == 't')
    {
	std::cout << "Simulation time is " << simulator->time << " seconds" << std::endl;
    }

    if (key == 'r')
    {
	std::cout << "reset simulation" << std::endl;
	simulator->reset();
    }
}


void RenderOpenGL::run()
{
   /* Start Event Processing Engine */  
  glutMainLoop();  
}

void RenderOpenGL::keyboardUpCallback(unsigned char key, int x, int y)
{

}

void RenderOpenGL::specialKeyboard(int key, int x, int y)
{

}

void RenderOpenGL::specialKeyboardUp(int key, int x, int y)
{

}

void RenderOpenGL::mouse(int button, int state, int x, int y)
{
    // add or remove a bit
    int bit = 1;
    if (button == GLUT_MIDDLE_BUTTON) bit = 2;
    if (button == GLUT_RIGHT_BUTTON) bit = 4;
    
    if (state == GLUT_DOWN)
    {
	mouseButton |= bit;
    }
    if (state == GLUT_UP)
    {
	mouseButton &= ~bit;
    }
}

void RenderOpenGL::mouseMotion(int x,int y)
{
    mouseLastX = mouseX;
    mouseLastY = mouseY;
    mouseX = x;
    mouseY = y;

    // a button is pressed ?
    if (mouseButton)
    {
	// left (1), middle (2) or right (4) mouse button is pressed
	dsMotion(mouseButton, mouseX - mouseLastX, mouseY - mouseLastY);
    }
}

void RenderOpenGL::displayCallback()
{
}

void RenderOpenGL::idle()
{
    // this render has to callback the simulator ...
   if (!paused)
   {

       simulationElapsedTicks += 1.0;
       while (simulationElapsedTicks >= simulationTicksDelay 
	      && simulationElapsedTicks > 0.0)
       {
	   simulator->step();
	   simulationElapsedTicks -= simulationTicksDelay;
       }
   }

   timeval tv;
   gettimeofday(&tv, NULL);   
   long int diff = (tv.tv_sec - lastTv.tv_sec) * 1000 * 1000 + (tv.tv_usec - lastTv.tv_usec);
   
   while (diff < refreshDelay)
   {          
       gettimeofday(&tv, NULL);   
       diff = (tv.tv_sec - lastTv.tv_sec) * 1000 * 1000 + (tv.tv_usec - lastTv.tv_usec);
       usleep (100);
   }

   drawScene();
   lastTv = tv;
}


// ===================
// drawing functions
// ===================

static void normalizeVector3 (float v[3])
{
    float len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (len <= 0.0f) {
	v[0] = 1;
	v[1] = 0;
	v[2] = 0;
    }
    else {
	len = 1.0f / (float)sqrt(len);
	v[0] *= len;
	v[1] *= len;
	v[2] *= len;
    }
}


//***************************************************************************
// OpenGL utility stuff

void RenderOpenGL::setCamera (float x, float y, float z, float h, float p, float r)
{
	
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glRotatef (90, 0,0,1);
  glRotatef (90, 0,1,0);
  glRotatef (r, 1,0,0);
  glRotatef (p, 0,1,0);
  glRotatef (-h, 0,0,1);
  glTranslatef (-x,-y,-z);
}


// sets the material color, not the light color

void RenderOpenGL::setColor (float r, float g, float b, float alpha)
{
  GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
  light_ambient[0] = r*0.95f;
  light_ambient[1] = g*0.95f;
  light_ambient[2] = b*0.95f;
  light_ambient[3] = alpha;
  light_diffuse[0] = r*0.95f;
  light_diffuse[1] = g*0.95f;
  light_diffuse[2] = b*0.95f;
  light_diffuse[3] = alpha;
  light_specular[0] = r*0.1f;
  light_specular[1] = g*0.1f;
  light_specular[2] = b*0.1f;
  light_specular[3] = alpha;
  // light_ambient[0] = r*0.3f;
  // light_ambient[1] = g*0.3f;
  // light_ambient[2] = b*0.3f;
  // light_ambient[3] = alpha;
  // light_diffuse[0] = r*0.7f;
  // light_diffuse[1] = g*0.7f;
  // light_diffuse[2] = b*0.7f;
  // light_diffuse[3] = alpha;
  // light_specular[0] = r*0.2f;
  // light_specular[1] = g*0.2f;
  // light_specular[2] = b*0.2f;
  // light_specular[3] = alpha;
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 3.0f);
}


void RenderOpenGL::setTransform (const float pos[3], const float R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[1];
  matrix[2]=R[2];
  matrix[3]=R[3];
  matrix[4]=R[4];
  matrix[5]=R[5];
  matrix[6]=R[6];
  matrix[7]=R[7];
  matrix[8]=R[8];
  matrix[9]=R[9];
  matrix[10]=R[10];
  matrix[11]=R[11];
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}


// set shadow projection transform

void RenderOpenGL::setShadowTransform()
{
  GLfloat matrix[16];
  for (int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-LIGHTX;
  matrix[9]=-LIGHTY;
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}


void RenderOpenGL::drawBox (const float sides[3])
{
  float lx = sides[0]*0.5f;
  float ly = sides[1]*0.5f;
  float lz = sides[2]*0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
  glNormal3f (-1,0,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (-lx,ly,lz);
  glNormal3f (0,1,0);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,ly,lz);
  glNormal3f (1,0,0);
  glVertex3f (lx,-ly,-lz);
  glVertex3f (lx,-ly,lz);
  glNormal3f (0,-1,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glEnd();

  // top face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,1);
  glTexCoord2f(1.0, 1.0);
  glVertex3f (-lx,-ly,lz); 

  glTexCoord2f(0.0, 1.0);
  glVertex3f (lx,-ly,lz);  

  glTexCoord2f(0.0, 0.0);
  glVertex3f (lx,ly,lz);   

  glTexCoord2f(1.0, 0.0);
  glVertex3f (-lx,ly,lz);  
  glEnd();

  // bottom face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,-1);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,-ly,-lz);
  glEnd();
}


// This is recursively subdivides a triangular area (vertices p1,p2,p3) into
// smaller triangles, and then draws the triangles. All triangle vertices are
// normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
// to be already normalized). Note this is not super-fast because it draws
// triangles rather than triangle strips.

void RenderOpenGL::drawPatch (float p1[3], float p2[3], float p3[3], int level)
{
  int i;
  if (level > 0) {
    float q1[3],q2[3],q3[3];		 // sub-vertices
    for (i=0; i<3; i++) {
      q1[i] = 0.5f*(p1[i]+p2[i]);
      q2[i] = 0.5f*(p2[i]+p3[i]);
      q3[i] = 0.5f*(p3[i]+p1[i]);
    }
    float length1 = (float)(1.0/sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
    float length2 = (float)(1.0/sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
    float length3 = (float)(1.0/sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
    for (i=0; i<3; i++) {
      q1[i] *= length1;
      q2[i] *= length2;
      q3[i] *= length3;
    }
    drawPatch (p1,q1,q3,level-1);
    drawPatch (q1,p2,q2,level-1);
    drawPatch (q1,q2,q3,level-1);
    drawPatch (q3,q2,p3,level-1);
  }
  else {
    glNormal3f (p1[0],p1[1],p1[2]);
    glVertex3f (p1[0],p1[1],p1[2]);
    glNormal3f (p2[0],p2[1],p2[2]);
    glVertex3f (p2[0],p2[1],p2[2]);
    glNormal3f (p3[0],p3[1],p3[2]);
    glVertex3f (p3[0],p3[1],p3[2]);
  }
}


// draw a sphere of radius 1


void RenderOpenGL::drawSphere()
{
  // icosahedron data for an icosahedron of radius 1.0
# define ICX 0.525731112119133606f
# define ICZ 0.850650808352039932f
  static GLfloat idata[12][3] = {
    {-ICX, 0, ICZ},
    {ICX, 0, ICZ},
    {-ICX, 0, -ICZ},
    {ICX, 0, -ICZ},
    {0, ICZ, ICX},
    {0, ICZ, -ICX},
    {0, -ICZ, ICX},
    {0, -ICZ, -ICX},
    {ICZ, ICX, 0},
    {-ICZ, ICX, 0},
    {ICZ, -ICX, 0},
    {-ICZ, -ICX, 0}
  };

  static int index[20][3] = {
    {0, 4, 1},	  {0, 9, 4},
    {9, 5, 4},	  {4, 5, 8},
    {4, 8, 1},	  {8, 10, 1},
    {8, 3, 10},   {5, 3, 8},
    {5, 2, 3},	  {2, 7, 3},
    {7, 10, 3},   {7, 6, 10},
    {7, 11, 6},   {11, 0, 6},
    {0, 1, 6},	  {6, 1, 10},
    {9, 0, 11},   {9, 11, 2},
    {9, 2, 5},	  {7, 2, 11},
  };

  GLuint listnum = 0;
  if (listnum==0) {
    listnum = glGenLists (1);
    glNewList (listnum,GL_COMPILE);
    glBegin (GL_TRIANGLES);
    for (int i=0; i<20; i++) {
      drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
		 &idata[index[i][0]][0],sphere_quality);
    }
    glEnd();
    glEndList();
  }
  glCallList (listnum);
  glDeleteLists(listnum, 1);
}


void RenderOpenGL::drawSphereShadow (float px, float py, float pz, float radius)
{
  // calculate shadow constants based on light vector
  static int init=0;
  static float len2,len1,scale;
  if (!init) {
    len2 = LIGHTX*LIGHTX + LIGHTY*LIGHTY;
    len1 = 1.0f/(float)sqrt(len2);
    scale = (float) sqrt(len2 + 1);
    init = 1;
  }

  // map sphere center to ground plane based on light vector
  px -= LIGHTX*pz;
  py -= LIGHTY*pz;

  const float kx = 0.96592582628907f;
  const float ky = 0.25881904510252f;
  float x=radius, y=0;

  glBegin (GL_TRIANGLE_FAN);
  for (int i=0; i<24; i++) {
    // for all points on circle, scale to elongated rotated shadow and draw
    float x2 = (LIGHTX*x*scale - LIGHTY*y)*len1 + px;
    float y2 = (LIGHTY*x*scale + LIGHTX*y)*len1 + py;
    glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
    glVertex3f (x2,y2,0);

    // rotate [x,y] vector
    float xtmp = kx*x - ky*y;
    y = ky*x + kx*y;
    x = xtmp;
  }
  glEnd();
}


void RenderOpenGL::drawTriangle (const float *v0, const float *v1, const float *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = v1[0] - v0[0];
  u[1] = v1[1] - v0[1];
  u[2] = v1[2] - v0[2];
  v[0] = v2[0] - v0[0];
  v[1] = v2[1] - v0[1];
  v[2] = v2[2] - v0[2];
  dCROSS (normal,=,u,v);
  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3fv (v0);
  glVertex3fv (v1);
  glVertex3fv (v2);
  glEnd();
}

void RenderOpenGL::drawTriangleD (const double *v0, const double *v1, const double *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = float( v1[0] - v0[0] );
  u[1] = float( v1[1] - v0[1] );
  u[2] = float( v1[2] - v0[2] );
  v[0] = float( v2[0] - v0[0] );
  v[1] = float( v2[1] - v0[1] );
  v[2] = float( v2[2] - v0[2] );
  dCROSS (normal,=,u,v);
  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3dv (v0);
  glVertex3dv (v1);
  glVertex3dv (v2);
  glEnd();
}


// draw a capped cylinder of length l and radius r, aligned along the x axis
void RenderOpenGL::drawCappedCylinder (float l, float r)
{
  int i,j;
  float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 =  ca*start_nx + sa*start_ny;
    float start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 = ca*start_nx - sa*start_ny;
    float start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  glPopMatrix();
}


// draw a cylinder of length l and radius r, aligned along the z axis

void RenderOpenGL::drawCylinder (float l, float r, float zoffset)
{
  int i;
  float tmp,ny,nz,a,ca,sa;
  const int n = 20;	// number of sides to the cylinder (divisible by 4)

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l+zoffset);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*r,nz*r,l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}


void RenderOpenGL::wrapCameraAngles()
{
    if (view_mode == 0)
    {
	for (int i=0; i<3; i++) 
	{
	    while (view_hpr[i] > 180) view_hpr[i] -= 360;
	    while (view_hpr[i] < -180) view_hpr[i] += 360;
	}
    }
    if (view_mode == 1)
    {
	while (view_dab[1] > 180) view_dab[1] -= 360;
	while (view_dab[1] < -180) view_dab[1] += 360;
	if (view_dab[2] < 0.0001) view_dab[2] = 0.0001;
	if (view_dab[2] > 179.999) view_dab[2] = 179.999;

	// also calculate cam pos... useful
	view_xyz[0] = view_lookat[0] + view_dab[0] * cos (view_dab[1] * DEG_TO_RAD) * sin (view_dab[2] * DEG_TO_RAD);
	view_xyz[1] = view_lookat[1] + view_dab[0] * sin (view_dab[1] * DEG_TO_RAD) * sin (view_dab[2] * DEG_TO_RAD);
	view_xyz[2] = view_lookat[2] + view_dab[0] * cos (view_dab[2] * DEG_TO_RAD);
    }
}


// call this to update the current camera position. the bits in `mode' say
// if the left (1), middle (2) or right (4) mouse button is pressed, and
// (deltax,deltay) is the amount by which the mouse pointer has moved.

// wheel : zoom in / out
// left : rotate around lookat point
// right : move the camera around the lookat point
// the lookat point is fixed for now, might change in the future
void RenderOpenGL::dsMotion (int mode, int deltax, int deltay)
{
    if (view_mode == 0)
    {
	float side = 0.05f * float(deltax);
	float fwd = (mode==4) ? (0.05f * float(deltay)) : 0.0f;
	float s = (float) sin (view_hpr[0]*DEG_TO_RAD);
	float c = (float) cos (view_hpr[0]*DEG_TO_RAD);
	
	if (mode==1) {
	    view_hpr[0] += float (deltax) * 1.0f;
	    view_hpr[1] += float (deltay) * 1.0f;
	}
	else {
	    view_xyz[0] += -s*side + c*fwd;
	    view_xyz[1] += c*side + s*fwd;
	    if (mode==2 || mode==5) view_xyz[2] += 0.05f * float(deltay);
	}
	wrapCameraAngles();
    }
    if (view_mode == 1)
    {
	// zoom
	if (mode == 2)
	{
	    float d = zoomCoefficient * float(deltay);
	    view_dab [0] += d;
	    if (view_dab [0] < 0.001) view_dab[0] = 0.001;
	}
	// move around
	if (mode == 1)
	{
	    float da = 0.4f * float(deltax);
	    float db = 0.4f * float(deltay);
	    
	    view_dab[1] -= da;
	    view_dab[2] -= db;
	}
	// more ?
	wrapCameraAngles();
    }
}




void RenderOpenGL::dsStopGraphics()
{
}


void RenderOpenGL::drawSky (float view_xyz[3])
{
  glDisable (GL_LIGHTING);
  /*
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    sky_texture->bind (0);
  }
  else {*/
    glDisable (GL_TEXTURE_2D);
    glColor3f (0,0.5,1.0);
  //}

  // make sure sky depth is as far back as possible
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glDepthRange (1,1);

  const float ssize = 1000.0f;
  static float offset = 0.0f;

  float x = ssize*sky_scale;
  float z = view_xyz[2] + sky_height;

  glBegin (GL_QUADS);
  glNormal3f (0,0,-1);
  glTexCoord2f (-x+offset,-x+offset);
  glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glTexCoord2f (-x+offset,x+offset);
  glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,x+offset);
  glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,-x+offset);
  glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glEnd();

  offset = offset + 0.002f;
  if (offset > 1) offset -= 1;

  glDepthFunc (GL_LESS);
  glDepthRange (0,1);
}


void RenderOpenGL::drawGround()
{
  glDisable (GL_LIGHTING);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  // glDepthRange (1,1);

/*
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (0);
  }
  else {
  */
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R,GROUND_G,GROUND_B);
  //}

  // ground fog seems to cause problems with TNT2 under windows
  /*
  GLfloat fogColor[4] = {0.5, 0.5, 0.5, 1};
  glEnable (GL_FOG);
  glFogi (GL_FOG_MODE, GL_EXP2);
  glFogfv (GL_FOG_COLOR, fogColor);
  glFogf (GL_FOG_DENSITY, 0.05f);
  glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
  glFogf (GL_FOG_START, 1.0);
  glFogf (GL_FOG_END, 5.0);
  */

  const float gsize = 100.0f;
  const float offset = -10.001f; // ... polygon offsetting doesn't work well

  glBegin (GL_QUADS);
  glNormal3f (0,0,1);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,gsize,offset);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,gsize,offset);
  glEnd();

  glDisable (GL_FOG);
}


void RenderOpenGL::drawPyramidGrid()
{
  // setup stuff
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);

  // draw the pyramid grid
  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      glPushMatrix();
      glTranslatef ((float)i,(float)j,(float)0);
      if (i==1 && j==0) setColor (1,0,0,1);
      else if (i==0 && j==1) setColor (0,0,1,1);
      else setColor (1,1,0,1);
      const float k = 0.03f;
      glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,-1,1);
      glVertex3f (0,0,k);
      glVertex3f (-k,-k,0);
      glVertex3f ( k,-k,0);
      glNormal3f (1,0,1);
      glVertex3f ( k, k,0);
      glNormal3f (0,1,1);
      glVertex3f (-k, k,0);
      glNormal3f (-1,0,1);
      glVertex3f (-k,-k,0);
      glEnd();
      glPopMatrix();
    }
  }
}


void RenderOpenGL::dsDrawFrame ()
{
//  if (current_state < 1) dsDebug ("internal error");
  current_state = 2;

  // setup stuff
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_TEXTURE_GEN_S);
  glDisable (GL_TEXTURE_GEN_T);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glEnable (GL_CULL_FACE);
  glCullFace (GL_BACK);
  glFrontFace (GL_CCW);

  // setup viewport
  glViewport (0,0,width,height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();
  // const float vnear = 0.1f;
  // const float vfar = 1000.0f;
  float k = aperture; 
  if (width >= height) {
    float k2 = float(height)/float(width);
    glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
  }
  else {
    float k2 = float(width)/float(height);
    glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
  }

  // setup lights. it makes a difference whether this is done in the
  // GL_PROJECTION matrix mode (lights are scene relative) or the
  // GL_MODELVIEW matrix mode (lights are camera relative, bad!).
  static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
  glColor3f (1.0, 1.0, 1.0);

  // clear the window
  glClearColor (0,0,0,1);
//  glClearColor (0.5,0.5,0.5,0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // snapshot camera position (in MS Windows it is changed by the GUI thread)
  float view2_xyz[3];
  float view2_hpr[3];
  memcpy (view2_xyz,view_xyz,sizeof(float)*3);
  memcpy (view2_hpr,view_hpr,sizeof(float)*3);
  if (view_mode == 0)
  {      
      // go to GL_MODELVIEW matrix mode and set the camera
      glMatrixMode (GL_MODELVIEW);
      glLoadIdentity();
      setCamera (view2_xyz[0],view2_xyz[1],view2_xyz[2],
		 view2_hpr[0],view2_hpr[1],view2_hpr[2]);
  }
  if (view_mode == 1)
  {
      // use dab to calculate xyz
      glMatrixMode (GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt (view2_xyz[0], view2_xyz[1], view2_xyz[2], view_lookat[0], view_lookat[1], view_lookat[2], view_up[0], view_up[1], view_up[2]);
  }
  
  // set the light position (for some reason we have to do this in model view.
  static GLfloat light_position[] = { LIGHTX, LIGHTY, 1.0, 0.0 };
  glLightfv (GL_LIGHT0, GL_POSITION, light_position);
  
  // draw the background (ground, sky etc)
//  drawSky (view2_xyz);
//  drawGround();

  // leave openGL in a known state - flat shaded white, no textures
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glColor3f (1,1,1);
  setColor (1,1,1,1);

  // draw the rest of the objects. set drawing state first.
  color[0] = 1;
  color[1] = 1;
  color[2] = 1;
  color[3] = 1;
  tnum = 0;
}


int RenderOpenGL::dsGetShadows()
{
  return use_shadows;
}


void RenderOpenGL::dsSetShadows (int a)
{
  use_shadows = (a != 0);
}


int RenderOpenGL::dsGetTextures()
{
  return use_textures;
}


void RenderOpenGL::dsSetTextures (int a)
{
  use_textures = (a != 0);
 // wood_texture->bind(1);
}

void RenderOpenGL::dsLoadNewTexture (char* filename) {
/*
    if( wood_texture )
    {
      delete wood_texture;
    }

    if (use_textures)
    {
        wood_texture = new Texture (filename);
    } else {
	wood_texture   = NULL;
    }        
    */ 
}
/*
void dsChangeTexture (Texture* texture) {
    if(use_textures)
    {
        if( *texture == *wood_texture )
	    return;

	if( wood_texture )
	    delete wood_texture;
	wood_texture = texture;
    } else {
        wood_texture   = NULL;
    }         
}
*/


//***************************************************************************
// C interface

// sets lighting and texture modes, sets current color


void RenderOpenGL::setupDrawingMode()
{
  glEnable (GL_LIGHTING);
  if (tnum) {
  /*  if (use_textures) {
      glEnable (GL_TEXTURE_2D);
      wood_texture->bind (1);
//       glEnable (GL_TEXTURE_GEN_S);
//       glEnable (GL_TEXTURE_GEN_T);
//       glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
//       glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
//       static GLfloat s_params[4] = {1.0f,0.0f,0.0f,1};
//       static GLfloat t_params[4] = {0.0f,1.0f,0.0f,1};
//       glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
//       glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
    }
    else {*/
      glDisable (GL_TEXTURE_2D);
    //}
  }
  else {
    glDisable (GL_TEXTURE_2D);
  }
  setColor (color[0],color[1],color[2],color[3]);

  if (color[3] < 1) {
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable (GL_BLEND);
  }
}


void RenderOpenGL::setShadowDrawingMode()
{
  glDisable (GL_LIGHTING);
 /* if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (1);
    glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    glEnable (GL_TEXTURE_2D);
    glEnable (GL_TEXTURE_GEN_S);
    glEnable (GL_TEXTURE_GEN_T);
    glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
    static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
    glTexGenfv (GL_S,GL_EYE_PLANE,s_params);
    glTexGenfv (GL_T,GL_EYE_PLANE,t_params);
  }
  else {*/
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	       GROUND_B*SHADOW_INTENSITY);
  //}
  glDepthRange (0,0.9999);
}


// void RenderOpenGL::dsSimulationLoop (int argc, char **argv,
// 				  int window_width, int window_height,
// 				  dsFunctions *fn)
// {
//   if (current_state != 0) dsError ("dsSimulationLoop() called more than once");
//   current_state = 1;

//   // look for flags that apply to us
//   int initial_pause = 0;
//   for (int i=1; i<argc; i++) {
//     if (strcmp(argv[i],"-notex")==0) use_textures = 0;
//     if (strcmp(argv[i],"-noshadow")==0) use_shadows = 0;
//     if (strcmp(argv[i],"-noshadows")==0) use_shadows = 0;
//     if (strcmp(argv[i],"-pause")==0) initial_pause = 1;
//   }

//   // if (fn->version > DS_VERSION)
//   //   dsDebug ("bad version number in dsFunctions structure");

//   initMotionModel();
//   dsPlatformSimLoop (window_width,window_height,fn,initial_pause);
//   current_state = 0;
// }


void RenderOpenGL::dsSetViewpoint (float xyz[3], float hpr[3])
{
//  if (current_state < 1) dsError ("dsSetViewpoint() called before simulation started");
  if (xyz) {
    view_xyz[0] = xyz[0];
    view_xyz[1] = xyz[1];
    view_xyz[2] = xyz[2];
  }
  if (hpr) {
    view_hpr[0] = hpr[0];
    view_hpr[1] = hpr[1];
    view_hpr[2] = hpr[2];
    wrapCameraAngles();
  }
  view_mode = 0;
}


void RenderOpenGL::dsGetViewpoint (float xyz[3], float hpr[3])
{
//  if (current_state < 1) dsError ("dsGetViewpoint() called before simulation started");
  if (xyz) {
    xyz[0] = view_xyz[0];
    xyz[1] = view_xyz[1];
    xyz[2] = view_xyz[2];
  }
  if (hpr) {
    hpr[0] = view_hpr[0];
    hpr[1] = view_hpr[1];
    hpr[2] = view_hpr[2];
  }
}

void RenderOpenGL::dsSetViewpoint2 (float lookat[3], float dab[3], float up[3])
{
//  if (current_state < 1) dsError ("dsSetViewpoint() called before simulation started");
  if (lookat) {
    view_lookat[0] = lookat[0];
    view_lookat[1] = lookat[1];
    view_lookat[2] = lookat[2];
  }
  if (dab) {
    view_dab[0] = dab[0];
    view_dab[1] = dab[1];
    view_dab[2] = dab[2];
    wrapCameraAngles();
  }
  if (up) {
    view_up[0] = up[0];
    view_up[1] = up[1];
    view_up[2] = up[2];
  }
  view_mode = 1;
}


void RenderOpenGL::dsGetViewpoint2 (float lookat[3], float dab[3], float up[3])
{
//  if (current_state < 1) dsError ("dsGetViewpoint() called before simulation started");
  if (lookat) {
    lookat[0] = view_lookat[0];
    lookat[1] = view_lookat[1];
    lookat[2] = view_lookat[2];
  }
  if (dab) {
    dab[0] = view_dab[0];
    dab[1] = view_dab[1];
    dab[2] = view_dab[2];
  }
  if (up) {
    up[0] = view_up[0];
    up[1] = view_up[1];
    up[2] = view_up[2];
  }
}


void RenderOpenGL::dsSetTexture (int texture_number)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  tnum = texture_number;
}


void RenderOpenGL::dsSetColor (float red, float green, float blue)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = 1;
}


void RenderOpenGL::dsSetColorAlpha (float red, float green, float blue,
				 float alpha)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = alpha;
}


void RenderOpenGL::dsDrawBox (const float pos[3], const float R[12],
			   const float sides[3])
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
       
    setupDrawingMode();
    glShadeModel (GL_FLAT);
    setTransform (pos,R);
    drawBox (sides);
    glPopMatrix();
    
  /*if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    glMatrixMode( GL_TEXTURE );
    setTransform (pos,R);
    wood_texture->bind(1);
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
  }
  else {*/
    glDisable (GL_TEXTURE_2D);
//  }

    if (use_shadows) 
    {
	setShadowDrawingMode();
	setShadowTransform();
	setTransform (pos,R);
	drawBox (sides);
	glPopMatrix();
	glPopMatrix();
	glDepthRange (0,1);
    }
}


void RenderOpenGL::dsDrawSphere (const float pos[3], const float R[12],
			      float radius)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glEnable (GL_NORMALIZE);
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  glScaled (radius,radius,radius);
  drawSphere();
  glPopMatrix();
  glDisable (GL_NORMALIZE);

  // draw shadows
  if (use_shadows) {
    glDisable (GL_LIGHTING);
 /*   if (use_textures) {
      ground_texture->bind (1);
      glEnable (GL_TEXTURE_2D);
      glDisable (GL_TEXTURE_GEN_S);
      glDisable (GL_TEXTURE_GEN_T);
      glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    }
    else {*/
      glDisable (GL_TEXTURE_2D);
      glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
		 GROUND_B*SHADOW_INTENSITY);
    //}
    glShadeModel (GL_FLAT);
    glDepthRange (0,0.9999);
    drawSphereShadow (pos[0],pos[1],pos[2],radius);
    glDepthRange (0,1);
  }
}


void RenderOpenGL::dsDrawTriangle (const float pos[3], const float R[12],
				const float *v0, const float *v1,
				const float *v2, int solid)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawTriangle (v0, v1, v2, solid);
  glPopMatrix();
}


void RenderOpenGL::dsDrawCylinder (const float pos[3], const float R[12],
				float length, float radius)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCylinder (length,radius,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCylinder (length,radius,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void RenderOpenGL::dsDrawCappedCylinder (const float pos[3], const float R[12],
				      float length, float radius)
{
//  if (current_state != 2) dsError ("drawing function called outside simulation loop");
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCappedCylinder (length,radius);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCappedCylinder (length,radius);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void RenderOpenGL::dsDrawLine (const float pos1[3], const float pos2[3])
{
  setupDrawingMode();
  glColor3f (color[0],color[1],color[2]);
  glDisable (GL_LIGHTING);
  glLineWidth (1);
  glShadeModel (GL_FLAT);
  glBegin (GL_LINES);
  glVertex3f (pos1[0],pos1[1],pos1[2]);
  glVertex3f (pos2[0],pos2[1],pos2[2]);
  glEnd();
}


void RenderOpenGL::dsDrawBoxD (const double pos[3], const double R[12],
		 const double sides[3])
{
  int i;
  float pos2[3],R2[12],fsides[3];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  for (i=0; i<3; i++) fsides[i]=(float)sides[i];
  dsDrawBox (pos2,R2,fsides);
}


void RenderOpenGL::dsDrawSphereD (const double pos[3], const double R[12], float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawSphere (pos2,R2,radius);
}


void RenderOpenGL::dsDrawTriangleD (const double pos[3], const double R[12],
				 const double *v0, const double *v1,
				 const double *v2, int solid)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];

  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos2,R2);
  drawTriangleD (v0, v1, v2, solid);
  glPopMatrix();
}


void RenderOpenGL::dsDrawCylinderD (const double pos[3], const double R[12],
		      float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCylinder (pos2,R2,length,radius);
}


void RenderOpenGL::dsDrawCappedCylinderD (const double pos[3], const double R[12],
			    float length, float radius)
{
  int i;
  float pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(float)pos[i];
  for (i=0; i<12; i++) R2[i]=(float)R[i];
  dsDrawCappedCylinder (pos2,R2,length,radius);
}


void RenderOpenGL::dsDrawLineD (const double _pos1[3], const double _pos2[3])
{
  int i;
  float pos1[3],pos2[3];
  for (i=0; i<3; i++) pos1[i]=(float)_pos1[i];
  for (i=0; i<3; i++) pos2[i]=(float)_pos2[i];
  dsDrawLine (pos1,pos2);
}


void RenderOpenGL::dsSetSphereQuality (int n)
{
  sphere_quality = n;
}


void RenderOpenGL::dsSetCappedCylinderQuality (int n)
{
  capped_cylinder_quality = n;
}

int RenderOpenGL::loadNetlogoMesh(std::string filename, float scaling, float rotAngle, float rotAxisX, float rotAxisY, float rotAxisZ)
{
    int displayList = glGenLists (1);
    glNewList(displayList, GL_COMPILE);

    glRotatef(rotAngle, rotAxisX, rotAxisY, rotAxisZ);
    glScalef(scaling, scaling, scaling);
    
    // here load the 3d model and draw the triangles
    std::ifstream file;
    file.open(filename.c_str());
    if (!file)
    {
	std::cout << "3D model could not be read from file !" << std::endl;
	return 0;
    }
    else
    {
	std::string lineread;
	typedef boost::tokenizer<boost::escaped_list_separator<char> > tokenizer;
	std::string se = "\\"; std::string ss = " \n"; std::string sq = "\"";
	boost::escaped_list_separator<char> separators(se, ss, sq);
	std::string stopToken = "stop";
	std::string normalToken = "normal:";

	float normalX, normalY, normalZ;
	float vertices[9];
	int vindex = 0;

	int parsingState = 0;
	int tmp = 0;
	while( std::getline(file, lineread) )
	{
	    tokenizer tokens(lineread, separators);
	    for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
	    {
		// first read number of shapes and some blabla
		if (parsingState < 3)
		{
		    parsingState++;
		}
		// until stop kw is encountered
		else
		{
		    if (*tok_iter == stopToken){ goto StopParsing;}

		    // read the normal
		    if (*tok_iter == normalToken)
		    {
			++tok_iter; normalX = atof (tok_iter->c_str());
			++tok_iter; normalY = atof (tok_iter->c_str());
			++tok_iter; normalZ = atof (tok_iter->c_str());		       			
		    }

		    // read vertex coordinates
		    else
		    {
			vertices[vindex++] = atof (tok_iter->c_str());
			if (vindex >= 9)
			{
			    vindex = 0;
			    glBegin(GL_TRIANGLES);
			    glNormal3f (normalX, normalY, normalZ);
			    glVertex3f (vertices[0], vertices[1], vertices[2]);
			    glVertex3f (vertices[3], vertices[4], vertices[5]);
			    glVertex3f (vertices[6], vertices[7], vertices[8]);
			    glEnd();			    			    
			}
		    }
		}
	    }
	}
    StopParsing : parsingState = 0;
    }
    file.close();

    glEndList();

    return displayList;
}
