/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2012 Alexandre Campo                                 */
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

#ifndef RENDER_OSG_H
#define RENDER_OSG_H

#include <cmath>
#include <list>
#include <sys/time.h>
#include <string>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/CameraManipulator>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>


#include "Service.h"


class RenderOSGInterface;

class RenderOSG : public Service
{
public :
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::observer_ptr<osgViewer::GraphicsWindow> window;
    osg::Group* root;
    osg::ref_ptr<osg::Camera> hudCamera;
    osg::ref_ptr<osgText::Font> font;
    
    int argc;
    char** argv;

    bool paused;
    bool fullscreen;

    int width;
    int height;
    
    // real world time to manage framerate
    timeval lastTv;
    long int refreshDelay;

    // interactions with user
    int mouseButton;
    int mouseLastX;
    int mouseLastY;
    int mouseX;
    int mouseY;
    
    float zoomCoefficient;

    // tuning visualisation speed
    float simulationTicksDelay;
    float simulationElapsedTicks;

    std::list<RenderOSGInterface*> objects;

    // hud
    osgText::Text* hudTimeText;
    osgText::Text* hudSpeedText;
    osgText::Text* hudPausedText;
    osgText::Text* hudHelpText;
    osg::Geode* hudTextGeode;
    std::string helpString;
    bool hudInfo = false;
    bool hudHelp = false;
    
    // Methods ==================

    RenderOSG(Simulator* simulator, int argc = 0, char** argv = 0);
    ~RenderOSG();

    void run();
    void drawScene();
    void step ();
    void setPaused (bool p);
    void recalculateTimings ();

    void loadFont(std::string filename);
    osgText::Text* createText(const osg::Vec3& pos, const std::string& content, float size );
    osgText::Text* createHudText(const osg::Vec3& pos, const std::string& content, float size );

    ///callback methods by glut
    void keyboard(unsigned char key, int x, int y);
    void keyboardUp(unsigned char key, int x, int y);
    void specialKeyboard(int key, int x, int y);
    void specialKeyboardUp(int key, int x, int y);
    void reshape(int w, int h);
    void mouse(int button, int state, int x, int y);
    void mouseMotion(int x,int y);
    void display();
    void idle();

};


#endif



