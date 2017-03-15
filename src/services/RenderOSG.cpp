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

using namespace std;

// Static callbacks
// ================

static RenderOSG* RenderOSGInstance = 0;


static void glutKeyboardCallback(unsigned char key, int x, int y)
{
    RenderOSGInstance->keyboard(key,x,y);
}

static void glutKeyboardUpCallback(unsigned char key, int x, int y)
{
    RenderOSGInstance->keyboardUp(key,x,y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y)
{
    RenderOSGInstance->specialKeyboard(key,x,y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y)
{
    RenderOSGInstance->specialKeyboardUp(key,x,y);
}


static void glutReshapeCallback(int w, int h)
{
    RenderOSGInstance->reshape(w,h);
}

static void glutIdleCallback()
{
    RenderOSGInstance->idle();
}

static void glutMouseCallback(int button, int state, int x, int y)
{
    RenderOSGInstance->mouse(button,state,x,y);
}


static void glutMotionCallback(int x,int y)
{
    RenderOSGInstance->mouseMotion(x,y);
}


static void glutDisplayCallback(void)
{
    RenderOSGInstance->display();
}

// =============================
// RenderOSG Methods
// =============================

void RenderOSG::display(void)
{
    // update and render the scene graph
    if (viewer.valid()) viewer->frame();

    // Swap Buffers
    glutSwapBuffers();
    glutPostRedisplay();
}

void RenderOSG::reshape( int w, int h )
{
    // update the window dimensions, in case the window has been resized.
    if (window.valid()) 
    {	
        if (height==0)	// Prevent A Divide By Zero If The Window Is Too Small
            height=1;
	
        this->width = w;
        this->height = h;

	window->resized(window->getTraits()->x, window->getTraits()->y, w, h);
        window->getEventQueue()->windowResize(window->getTraits()->x, window->getTraits()->y, w, h);
	hudCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, w, 0, h));
    }
}

void RenderOSG::mouse( int button, int state, int x, int y )
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

void RenderOSG::mouseMotion( int x, int y )
{
    if (window.valid())
    {	
	mouseLastX = mouseX;
	mouseLastY = mouseY;
	mouseX = x;
	mouseY = y;

	window->getEventQueue()->mouseMotion( x, y );
    }
}

void RenderOSG::keyboard( unsigned char key, int /*x*/, int /*y*/ )
{
    /* avoid thrashing this call */
    usleep(50);

    osg::Matrix initialCamMatrix;

    switch (key)
    {
    case ' ' :

	setPaused(!paused);
	
	cout << "Pause toggled ... " << endl;
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
	cout << "Accelerate and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << endl;
	break;

    case '-' : 

	// that reduce delay
	simulationTicksDelay *= 1.5;
	cout << "Slow down and run at " << 1.0 / simulationTicksDelay << " speed WRT realtime" << endl;
	break;
    

    case BACKSPACE : 

	simulationTicksDelay = 1.0;
	cout << "reset speed and run realtime" << endl;
	break;

    case 'h' : 
	
	hudHelp = !hudHelp;

	if (hudHelp)
	    hudHelpText->setText(helpString);
	else
	    hudHelpText->setText(string(""));
	    
	break;
	
    case 'i' : 

	hudInfo = !hudInfo;
	hudCamera->setNodeMask(hudInfo);
       
	break;
	
    case 'r' : 

	cout << "reset simulation" << endl;
	simulator->reset();
	break;

    case 'z' : 

	cout << "reset view" << endl;
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
    glutInit(&argc, argv);  
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

    width = 800;
    height = 600;
    glutInitWindowSize(width, height);  

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

    osg::setNotifyLevel( osg::FATAL );
  
    // create the view of the scene.
    viewer = new osgViewer::Viewer;
    window = viewer->setUpViewerAsEmbeddedInWindow(100,100,width,height);
    root = new osg::Group ();
    viewer->setSceneData(root);

    // use a spehrical manipulator to move around
    osg::ref_ptr<osgGA::SphericalManipulator> manip = new osgGA::SphericalManipulator;
    manip->setAllowThrow(false);
    viewer->setCameraManipulator(manip);
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


    // add HUD
    hudCamera = new osg::Camera;
    hudCamera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    hudCamera->setClearMask( GL_DEPTH_BUFFER_BIT );
    hudCamera->setRenderOrder( osg::Camera::POST_RENDER );
    hudCamera->setAllowEventFocus( false );
    hudCamera->setProjectionMatrix( osg::Matrix::ortho2D(0, width, 0, height) );
    hudCamera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    root->addChild( hudCamera.get() );
    
    // add some text to display in hud
    hudTimeText = createHudText(osg::Vec3(20.0f, 80.0f, 0.0f), "Time : ", 20.0f);
    hudSpeedText = createHudText(osg::Vec3(20.0f, 50.0f, 0.0f), "Speed : ", 20.0f);
    hudPausedText = createHudText(osg::Vec3(20.0f, 20.0f, 0.0f), "", 20.0f);
    osg::Geode* hudTextGeode = new osg::Geode;
    hudTextGeode->addDrawable( hudTimeText );
    hudTextGeode->addDrawable( hudSpeedText );
    hudTextGeode->addDrawable( hudPausedText );
    hudCamera->addChild( hudTextGeode );    
    
    viewer->realize();
    
    // display some basic information for end user
    helpString += string("The following shortcuts are available :\n");
    helpString += string("---------------------------------------\n");
    helpString += string("      escape : quit the program\n");
    helpString += string("       space : toggle pause\n");
    helpString += string("           r : reset simulation\n");
    helpString += string("           - : slow down simulation\n");
    helpString += string("      + or = : accelerate simulation\n");
    helpString += string("   backspace : real time speed\n");
    helpString += string("           i : toggle hud info\n");
    helpString += string("           h : toggle help\n");
    helpString += string(" \n");

    cout << "OpenSceneGraph render is active" << endl << endl;
    cout << helpString;
    
    hudHelpText = createHudText(osg::Vec3(20.0f, 400.0f, 0.0f), "", 20.0f);
    hudTextGeode->addDrawable( hudHelpText );    
    
    // activate only after text is created...
    hudInfo = true;
}

void RenderOSG::loadFont(string filename)
{
    font = osgText::readFontFile(filename);
    hudTimeText->setFont(font);
    hudSpeedText->setFont(font);
    hudPausedText->setFont(font);
    hudHelpText->setFont(font);
}

osgText::Text* RenderOSG::createHudText(const osg::Vec3& pos, const string& content, float size)
{
    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setDataVariance( osg::Object::DYNAMIC );
    text->setFont( font.get() );
    text->setCharacterSize( size );
    text->setAxisAlignment( osgText::TextBase::XY_PLANE );
    text->setPosition( pos );
    text->setText( content );
    return text.release();
}

osgText::Text* RenderOSG::createText(const osg::Vec3& pos, const string& content, float size)
{
    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    text->setDataVariance( osg::Object::DYNAMIC );
    text->setFont( font.get() );
    text->setCharacterSize( size );
    text->setAxisAlignment( osgText::TextBase::SCREEN );
    text->setPosition( pos );
    text->setText( content );
    return text.release();
}

RenderOSG::~RenderOSG ()
{

}

// if timestep is changed, we have to call that function
void RenderOSG::recalculateTimings ()
{
    simulationTicksDelay = simulator->timestep / (1.0 / 24.0);
}

void RenderOSG::step ()
{
}

void RenderOSG::drawScene ()
{
    for (auto* o : objects)
    	o->draw(this);

    if (hudInfo)
    {
	hudTimeText->setText(string("Time : ") + to_string(simulator->time));
	hudSpeedText->setText(string("Speed : ") + to_string(1.0 / simulationTicksDelay));
    }
}

void RenderOSG::setPaused (bool p)
{
    paused = p;
    
    if (paused)
	hudPausedText->setText(string("PAUSED"));
    else
	hudPausedText->setText(string(""));	
}

void RenderOSG::run()
{
    glutMainLoop();  
}

void RenderOSG::keyboardUp(unsigned char key, int x, int y)
{

}

void RenderOSG::specialKeyboard(int key, int x, int y)
{

}

void RenderOSG::specialKeyboardUp(int key, int x, int y)
{

}

void RenderOSG::idle()
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


