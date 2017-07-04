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

#ifndef A_FISH_H
#define A_FISH_H

#include "Object.h"
#include "RenderOpenGLInterface.h"
#include "RenderOSGInterface.h"
#include "PhysicsBulletInterface.h"
#include "WaterVolumeInterface.h"
#include "EnergyManagerInterface.h"

//#include "DeviceMagicForce.h"
#include "DevicePropeller.h"
#include "DeviceBallast.h"
#include "DeviceAcousticTransceiver.h"
#include "DeviceOpticalTransceiver.h"
#include "DeviceRayCast.h"
#include "DeviceNetworker.h"
#include "DeviceElectricSense.h"


// store only one and same geometry  for all robots
static osg::ref_ptr<osg::Node> aFishNode = NULL;

class aFish : public virtual Object, public RenderOpenGLInterface, public PhysicsBulletInterface, public EnergyManagerInterface, public RenderOSGInterface, public WaterVolumeInterface
{
public:
    
    static const int FRONT_LEFT = 0;
    static const int FRONT_RIGHT = 1;
    static const int LEFT = 2;
    static const int RIGHT = 3;
    static const int FRONT = 4;
    static const int BACK = 5;
    static const int TOP = 6;
    static const int BOTTOM = 7;
    
    float dimensions[3];
    float mass;

    btCollisionShape* shape;
    btRigidBody* body;

    btTransform principalTransform;
    btTransform principalTransformInverse;

    GLuint displayList;
    GLuint cube;             // storage for the display list
    GLuint top;              // storage for the 2nd display list
    GLuint xloop;            // loop for x axis.
    GLuint yloop;            // loop for y axis.
    
    GLfloat xrot;            // rotates cube on the x axis.
    GLfloat yrot;            // rotates cube on the y axis.
    
    std::string meshFilename;
    osgText::Text* text;
    osg::ref_ptr<osg::Geode> textGeode;
    
    DevicePropeller* propellerLeft;
    DevicePropeller* propellerRight;
    DeviceBallast* ballast;
    DeviceRayCast* rayFrontLU; // left up
    DeviceRayCast* rayFrontLD; // left down
    DeviceRayCast* rayFrontRU; // right up
    DeviceRayCast* rayFrontRD; // right down
    DeviceRayCast* rayTop;
    DeviceRayCast* rayBottom;
    DeviceRayCast* rayLeft;
    DeviceRayCast* rayRight;
    DeviceRayCast* rayBack;
    DeviceAcousticTransceiver* acoustic;
    DeviceOpticalTransceiver* optical;
    DeviceNetworker* networker;
    DeviceElectricSense* esense;
    
    float colr, colg, colb, cola;

    aFish ();
    ~aFish ();

    void step ();

    void addDevices();

    void draw (RenderOpenGL* r);
    void draw (RenderOSG* r);

    void registerService (PhysicsBullet* p);
    void unregisterService (PhysicsBullet* p);

    void registerService (RenderOpenGL* r);
    void unregisterService (RenderOpenGL* r);

    void setMeshFilename (std::string filename);
    void registerService (RenderOSG* r);
    void unregisterService (RenderOSG* r);

    void registerService (EnergyManager* m);
    void unregisterService (EnergyManager* m);

    void registerService (WaterVolume* w);
    void unregisterService (WaterVolume* w);

    void setPosition (btVector3 p);
    btVector3 getPosition ();
    void setRotation (btQuaternion q);
    btQuaternion getRotation (btQuaternion q);

    void setColor (float r = 1, float g = 0, float b = 0, float a = 1);

    void setTextDrawable(bool d);
    void setText(std::string s);
    void setTextColor(float r, float g, float b, float a = 1);

    void setProximitySensorsRange(float range);


    // ================ Custom Physics ==============================
    //
    // extended calculations for modelling underwater dynamics
    
    btVector3 m_linearDrag;
    btVector3 m_angularDrag;
    btVector3 m_linearQuadraticDrag;
    btVector3 m_angularQuadraticDrag;

    btVector3 m_centerOfVolume;

    btVector3 m_linearAddedMass;
    btVector3 m_angularAddedMass;
    
    btVector3 m_inertia;
    
    void setAddedMass (const btVector3& linear, const btVector3& angular);
    void setDragCoefficients (const btVector3& linear, const btVector3& angular);
    void setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular, float fluidDensity);
    void updateInertiaTensor ();
};

#endif
