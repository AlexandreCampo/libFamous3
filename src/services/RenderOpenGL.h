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

// code of this render is adapted from drawstuff lib of ODE. An ugly tool, but quickly set up. 

#ifndef RENDER_OPENGL_H
#define RENDER_OPENGL_H

#include <cmath>
#include <list>
#include <sys/time.h>
#include <string>

#include "Service.h"

// TODO in principle this service should not deal directly with the physics service, but I am too lazy to implement some feedback in the objects...
#include "PhysicsBullet.h"

class RenderOpenGLInterface;

class RenderOpenGL : public Service
{
public :
    int argc;
    char** argv;

    bool paused;
    bool fullscreen;

    int width;
    int height;
    float aperture;
    float vnear;
    float vfar;

    int window;
    
    // real world time to manage framerate
    timeval lastTv;
    long int refreshDelay;

    float ground_scale;
    float ground_ofsx;
    float ground_ofsy;
    float sky_scale;
    float sky_height;

    float color[4];
    int tnum;
    int sphere_quality;
    int capped_cylinder_quality;

    float view_xyz[3];	// position x,y,z
    float view_hpr[3];	// heading, pitch, roll (degrees)

    float view_lookat[3]; // lookat point
    float view_dab[3]; // distance + alpha / beta angles to map cam pos on a sphere
    float view_up[3]; // the up vector for gluLookat, typically 0,0,1; but not always ...
    int view_mode;

    int current_state;
    int use_textures;		// 1 if textures to be drawn
    int use_shadows;		// 1 if shadows to be drawn
    

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

    std::list<RenderOpenGLInterface*> objects;
    
   
    // Methods ==================

    RenderOpenGL(int argc = 0, char** argv = 0);
    ~RenderOpenGL();

    /* void this->add (RenderOpenGLInterface* object); */
    /* void Remove (RenderOpenGLInterface* object); */

    void run();
    void initGL(int width, int height);
    void resizeScene(int width, int height);
    void drawScene();
    void step ();
    void setPaused (bool p);
    void recalculateTimings ();

    ///callback methods by glut
    void keyboardCallback(unsigned char key, int x, int y);
    void keyboardUpCallback(unsigned char key, int x, int y);
    void specialKeyboard(int key, int x, int y);
    void specialKeyboardUp(int key, int x, int y);
    void reshape(int w, int h);
    void mouse(int button, int state, int x, int y);
    void mouseMotion(int x,int y);
    void displayCallback();
    void idle();

    
    // drawing methods
    void setPerspective (float aperture, float near, float far);
    void setCamera (float x, float y, float z, float h, float p, float r);
    void setColor (float r, float g, float b, float alpha);
    void setTransform (const float pos[3], const float R[12]);
    void setShadowTransform();
    void drawBox (const float sides[3]);
    void drawPatch (float p1[3], float p2[3], float p3[3], int level);
    void drawSphere();
    void drawSphereShadow (float px, float py, float pz, float radius);
    void drawTriangle (const float *v0, const float *v1, const float *v2, int solid);
    void drawTriangleD (const double *v0, const double *v1, const double *v2, int solid);
    void drawCappedCylinder (float l, float r);
    void drawCylinder (float l, float r, float zoffset);
    void wrapCameraAngles();
    void dsMotion (int mode, int deltax, int deltay);
    void dsStopGraphics();
    void drawSky (float view_xyz[3]);
    void drawGround();
    void drawPyramidGrid();
    void dsDrawFrame ();
    int dsGetShadows();
    void dsSetShadows (int a);
    int dsGetTextures();
    void dsSetTextures (int a);
    void dsLoadNewTexture (char* filename);
    void setupDrawingMode();
    void setShadowDrawingMode();
    void dsSetViewpoint (float xyz[3], float hpr[3]);
    void dsGetViewpoint (float xyz[3], float hpr[3]);
    void dsSetViewpoint2 (float lookat[3], float dab[3], float up[3] = NULL);
    void dsGetViewpoint2 (float lookat[3], float dab[3], float up[3] = NULL);
    void dsSetTexture (int texture_number);
    void dsSetColor (float red, float green, float blue);
    void dsSetColorAlpha (float red, float green, float blue, float alpha);
    void dsDrawBox (const float pos[3], const float R[12], const float sides[3]);
    void dsDrawSphere (const float pos[3], const float R[12], float radius);
    void dsDrawTriangle (const float pos[3], const float R[12], const float *v0, const float *v1, const float *v2, int solid);
    void dsDrawCylinder (const float pos[3], const float R[12], float length, float radius);
    void dsDrawCappedCylinder (const float pos[3], const float R[12], float length, float radius);
    void dsDrawLine (const float pos1[3], const float pos2[3]);
    void dsDrawBoxD (const double pos[3], const double R[12], const double sides[3]);
    void dsDrawSphereD (const double pos[3], const double R[12], float radius);
    void dsDrawTriangleD (const double pos[3], const double R[12], const double *v0, const double *v1, const double *v2, int solid);
    void dsDrawCylinderD (const double pos[3], const double R[12], float length, float radius);
    void dsDrawCappedCylinderD (const double pos[3], const double R[12], float length, float radius);
    void dsDrawLineD (const double _pos1[3], const double _pos2[3]);
    void dsSetSphereQuality (int n);
    void dsSetCappedCylinderQuality (int n);

    int loadNetlogoMesh(std::string filename, float scaling = 1.0, float rotAngle = 0.0, float rotAxisX = 0.0, float rotAxisY = 0.0, float rotAxisZ = 0.0);
};


// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

// light vector. LIGHTZ is implicitly 1
#define LIGHTX (1.0f)
#define LIGHTY (0.4f)

// ground and sky
#define SHADOW_INTENSITY (0.65f)
#define GROUND_R (0.5f) 	// ground color for when there's no texture
#define GROUND_G (0.5f)
#define GROUND_B (0.3f)

#define dCROSS(a,op,b,c)				\
    (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]);		\
       (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]);	\
       (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);

inline float dDOT (const float *a, const float *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }


typedef double dsRotationMatrixD[12];
typedef float dsRotationMatrix[12];

// Create a drawstuff rotation matrix corresponding to a given angle:
#define dRotation2dsRotationMatrix(angle, matrix)                                                               \
  {                                                                                                             \
     matrix[0 * 4 + 2] = matrix[1 * 4 + 2] = matrix[2 * 4 + 0] = matrix[2 * 4 + 1] = 0;                         \
     matrix[0 * 4 + 3] = matrix[1 * 4 + 3] = matrix[2 * 4 + 3] = 0;                                             \
     matrix[0 * 4 + 0] = matrix[1 * 4 + 1] = cos(angle);                                                        \
     matrix[0 * 4 + 1] = -sin(angle);                                                                           \
     matrix[1 * 4 + 0] = sin(angle);                                                                            \
     matrix[2 * 4 + 2] = 1;					\
  }


#define dTwoAngleRotation2dsRotationMatrix(z, y, matrix)                                                                                           \
  {                                                                                                                                                \
     matrix[0 * 4 + 0] = cos(y)*cos(z);           matrix[0 * 4 + 1] = -sin(z)        ;  matrix[0 * 4 + 2] = sin(y)*cos(z);         matrix[0 * 4 + 3] = 0;   \
     matrix[1 * 4 + 0] = cos(y)*sin(z);           matrix[1 * 4 + 1] = cos(z)         ;  matrix[1 * 4 + 2] = sin(y)*sin(z);         matrix[1 * 4 + 3] = 0;   \
     matrix[2 * 4 + 0] = -sin(y);                 matrix[2 * 4 + 1] = 0              ;  matrix[2 * 4 + 2] = cos(y);                matrix[2 * 4 + 3] = 0;   \
  }


#endif



