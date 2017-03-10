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

    GLuint displayList;
    GLuint cube;             // storage for the display list
    GLuint top;              // storage for the 2nd display list
    GLuint xloop;            // loop for x axis.
    GLuint yloop;            // loop for y axis.
    
    GLfloat xrot;            // rotates cube on the x axis.
    GLfloat yrot;            // rotates cube on the y axis.
    
    std::string meshFilename;

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

    float colr, colg, colb, cola;

    aFish ();
    ~aFish ();

    void Step ();

    void AddDevices();

    void Draw (RenderOpenGL* r);
    void Draw (RenderOSG* r);

    void Register (PhysicsBullet* p);
    void Unregister (PhysicsBullet* p);

    void Register (RenderOpenGL* r);
    void Unregister (RenderOpenGL* r);

    void SetMeshFilename (std::string filename);
    void Register (RenderOSG* r);
    void Unregister (RenderOSG* r);

    void Register (EnergyManager* m);
    void Unregister (EnergyManager* m);

    void Register (WaterVolume* w);
    void Unregister (WaterVolume* w);

    void SetPosition (btVector3 p);
    btVector3 GetPosition ();
    void SetRotation (btQuaternion q);
    btQuaternion GetRotation (btQuaternion q);

    void SetColor (float r = 1, float g = 0, float b = 0, float a = 1);

    void SetProximitySensorsRange(float range);


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
