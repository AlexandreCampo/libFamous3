/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2015 Alexandre Campo                                 */
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


#include "RenderOSGInterface.h"
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

#include <osg/Config>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/CameraManipulator>
#include <osgDB/ReadFile>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/StateSet>
#include <osg/PositionAttitudeTransform>


#include "RenderOSG.h"

#define ESCAPE 27
#define BACKSPACE 8


// Static callbacks
// ================

static RenderOSG* RenderOSGInstance = 0;


static void glutKeyboardCallback(unsigned char key, int x, int y)
{
    RenderOSGInstance->Keyboard(key,x,y);
}

static void glutKeyboardUpCallback(unsigned char key, int x, int y)
{
    RenderOSGInstance->KeyboardUp(key,x,y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y)
{
    RenderOSGInstance->SpecialKeyboard(key,x,y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y)
{
    RenderOSGInstance->SpecialKeyboardUp(key,x,y);
}


static void glutReshapeCallback(int w, int h)
{
    RenderOSGInstance->Reshape(w,h);
}

static void glutIdleCallback()
{
    RenderOSGInstance->Idle();
}

static void glutMouseCallback(int button, int state, int x, int y)
{
    RenderOSGInstance->Mouse(button,state,x,y);
}


static void glutMotionCallback(int x,int y)
{
    RenderOSGInstance->MouseMotion(x,y);
}


static void glutDisplayCallback(void)
{
    RenderOSGInstance->Display();
}

// =============================
// RenderOSG Methods
// =============================

void RenderOSG::Display(void)
{
    // update and render the scene graph
    if (viewer.valid()) viewer->frame();

    // Swap Buffers
    glutSwapBuffers();
    glutPostRedisplay();
}

void RenderOSG::Reshape( int w, int h )
{
    // update the window dimensions, in case the window has been resized.
    if (window.valid()) 
    {
        window->resized(window->getTraits()->x, window->getTraits()->y, w, h);
        window->getEventQueue()->windowResize(window->getTraits()->x, window->getTraits()->y, w, h );
	
        if (height==0)				// Prevent A Divide By Zero If The Window Is Too Small
            height=1;
	
        this->width = w;
        this->height = h;
	
//  glViewport(0, 0, width, height);		// Reset The Current Viewport And Perspective Transformation
	
//  setPerspective (aperture, vnear, vfar);
    }
}

void RenderOSG::Mouse( int button, int state, int x, int y )
{
    if (window.valid())
    {
        if (state==GLUT_DOWN) window->getEventQueue()->mouseButtonPress( x, y, button+1 );
        else if (state == GLUT_UP) window->getEventQueue()->mouseButtonRelease( x, y, button+1 );


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
}

void RenderOSG::MouseMotion( int x, int y )
{
    if (window.valid())
    {	
	mouseLastX = mouseX;
	mouseLastY = mouseY;
	mouseX = x;
	mouseY = y;

//	if (mouseButton)
	window->getEventQueue()->mouseMotion( x, y );

//	cout << "mouse " << x << " " << y << endl;
    }
}

void RenderOSG::Keyboard( unsigned char key, int /*x*/, int /*y*/ )
{
    /* avoid thrashing this call */
    usleep(100);

    osg::Matrix initialCamMatrix;

    switch (key)
    {
    case ' ' :

	if (paused) 
	{
	    paused = false;
	}
	else
	{
	    paused = true;
	}
	std::cout << "Pause toggled ... " << std::endl;
	break;

    /* If escape is pressed, kill everything. */
    case ESCAPE : 

	glutDestroyWindow(glutGetWindow());
	delete simulator;
	exit(0);                   

    // increase simulation speed
    case '=' :
    case '+' :

	// that reduce delay
	simulationTicksDelay /= 1.5;
//	if (simulationTicksDelay < 1.0) simulationTicksDelay = 1.0;
	std::cout << "Accelerate and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << std::endl;
	break;

    case '-' : 

	// that reduce delay
	simulationTicksDelay *= 1.5;
	std::cout << "Slow down and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << std::endl;
	break;
    

    case BACKSPACE : 

	simulationTicksDelay = 1.0;
	std::cout << "Reset speed and run realtime" << std::endl;
	break;

    case 't' : 

	std::cout << "Simulation time is " << simulator->time << " seconds" << std::endl;
	break;
	
    case 'r' : 

	std::cout << "Reset simulation" << std::endl;
	simulator->Reset();
	break;

    case 'h' : 

	std::cout << "Reset view" << std::endl;
	initialCamMatrix.makeLookAt (
		osg::Vec3(0, 0, 10), // eye position 
		osg::Vec3(0, 0, 0), // looking at position
		osg::Vec3(0, 1, 0) // up vector
	    );
	viewer->getCameraManipulator()->setByMatrix(initialCamMatrix);
	break;

    default:
	if (window.valid())
	{
	    window->getEventQueue()->keyPress( (osgGA::GUIEventAdapter::KeySymbol) key );
	    window->getEventQueue()->keyRelease( (osgGA::GUIEventAdapter::KeySymbol) key );
	}
	break;
    }
}


RenderOSG::RenderOSG(Simulator* simulator, int argc, char** argv)
{
    RenderOSGInstance = this;

    this->simulator = simulator;
    this->argc = argc;
    this->argv = argv;

    // ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
    // ground_ofsx = 0.5;		// offset of ground texture
    // ground_ofsy = 0.5;
    // sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
    // sky_height = 1.0f;		// sky height above viewpoint
    // color[0] = 0;
    // color[1] = 0;
    // color[2] = 0;
    // color[3] = 0;
    // sphere_quality = 1;
    // capped_cylinder_quality = 3;
    // current_state = 0;
    // use_textures=0;		// 1 if textures to be drawn
    // use_shadows=0;		// 1 if shadows to be drawn

    // view_xyz[0] = 0;	// position x,y,z
    // view_xyz[1] = 0;	// position x,y,z
    // view_xyz[2] = 20.0;	// position x,y,z
    // view_hpr[0] = 0;	// heading, pitch, roll (degrees)
    // view_hpr[1] = -90.0;	// heading, pitch, roll (degrees)
    // view_hpr[2] = 0;	// heading, pitch, roll (degrees)
    // view_up[0] = 0;	// up vector
    // view_up[1] = 0;	// up vector
    // view_up[2] = 1;	// up vector

    // aperture = 45.0 * M_PI / 180.0;
    // vnear = 1.0;
    // vfar = 1000.0;

    // // look at 0,0,0
    // view_lookat[0] = 0.0;
    // view_lookat[1] = 0.0;
    // view_lookat[2] = 0.0;
    
    // // camera is at distance, alpha, beta
    // view_dab[0] = 10.0;
    // view_dab[1] = 0.0;
    // view_dab[2] = 45.0;

    // view_mode = 1;

    mouseButton = 0;
    mouseLastX = 0;
    mouseLastY = 0;
    mouseX = 0;
    mouseY = 0;

    zoomCoefficient = 0.2;

    simulationElapsedTicks = 0.0;
    refreshDelay = (long int) (1.0 / 24.0 * 1000.0 * 1000.0);
    gettimeofday(&lastTv, NULL);   
    RecalculateTimings();

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
  width = 800;
  height = 600;

  /* the window starts at the upper left corner of the screen */
  glutInitWindowPosition(100, 100);  

  /* Open a window */  
  glutCreateWindow("Simulator");  

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
//  InitGL(800, 600);

  // create the view of the scene.
  viewer = new osgViewer::Viewer;
  window = viewer->setUpViewerAsEmbeddedInWindow(100,100,800,600);
//    viewer->setSceneData(loadedModel.get());
  root = new osg::Group ();
  viewer->setSceneData(root);
//  osg::ref_ptr<osgGA::TrackballManipulator> manip = new osgGA::TrackballManipulator;
  osg::ref_ptr<osgGA::SphericalManipulator> manip = new osgGA::SphericalManipulator;
  manip->setAllowThrow(false);
  viewer->setCameraManipulator(manip);
//    viewer->addEventHandler(new osgViewer::StatsHandler);
  viewer->getCamera()->setClearColor(osg::Vec4(0,0,0,1));


  // add one light to the scene
  osg::ref_ptr<osg::Light> light = new osg::Light();
  light->setPosition(osg::Vec4(1, 0.4, 1.0, 0.0));
  light->setDiffuse(osg::Vec4(1,1,1,1));
  light->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
  light->setAmbient(osg::Vec4(0.5, 0.5, 0.5, 1.0));

  osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
  lightSource->setLight(light); 

  lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
  osg::StateSet* lightStateSet = root->getOrCreateStateSet();
  lightSource->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);
  root->addChild(lightSource);

  viewer->realize();
  
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

RenderOSG::~RenderOSG ()
{

}

// if timestep is changed, we have to call that function
void RenderOSG::RecalculateTimings ()
{
    simulationTicksDelay = simulator->timestep / (1.0 / 24.0);
}

void RenderOSG::Step ()
{
}

void RenderOSG::DrawScene ()
{
    for (auto* o : objects)
    	o->Draw(this);
}

void RenderOSG::SetPaused (bool p)
{
    paused = p;
}

void RenderOSG::Run()
{
    glutMainLoop();  
}

void RenderOSG::KeyboardUp(unsigned char key, int x, int y)
{

}

void RenderOSG::SpecialKeyboard(int key, int x, int y)
{

}

void RenderOSG::SpecialKeyboardUp(int key, int x, int y)
{

}

void RenderOSG::Idle()
{
    // this render has to callback the simulator ...
   if (!paused)
   {

       simulationElapsedTicks += 1.0;
       while (simulationElapsedTicks >= simulationTicksDelay 
	      && simulationElapsedTicks > 0.0)
       {
	   simulator->Step();
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

   DrawScene();

   lastTv = tv;
}


