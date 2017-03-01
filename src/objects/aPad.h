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

#ifndef A_PAD_H
#define A_PAD_H

#include "Object.h"
#include "RenderOSGInterface.h"
#include "PhysicsBulletInterface.h"
#include "WaterVolumeInterface.h"

#include "DeviceAcousticTransceiver.h"
#include "DeviceBallast.h"
#include "DevicePropeller.h"

// store only one and same geometry  for all robots
static osg::ref_ptr<osg::Node> aPadNode = NULL;

class aPad : public virtual Object, public PhysicsBulletInterface, public RenderOSGInterface, public WaterVolumeInterface
{
public:

    float dimensions[3];
    float mass;

    btCollisionShape* shape;
    btRigidBody* body;

    btTransform principalTransform;

    
    std::string meshFilename;

    DevicePropeller* propellerLeft;
    DevicePropeller* propellerRight;

    std::vector<DeviceBallast*> ballasts;
    
    int acousticCollisionFilter;
    int acousticCollisionType;
    DeviceAcousticTransceiver* acoustic;

    float colr, colg, colb, cola;

    aPad ();
    ~aPad ();

    void Step ();

    void AddDevices();

    void Draw (RenderOSG* r);

    void Register (PhysicsBullet* p);
    void Unregister (PhysicsBullet* p);

    void SetMeshFilename (std::string filename);
    void Register (RenderOSG* r);
    void Unregister (RenderOSG* r);

    void Register (WaterVolume* w);
    void Unregister (WaterVolume* w);

    void SetPosition (btVector3 p);
    btVector3 GetPosition ();
    void SetRotation (btQuaternion q);
    btQuaternion GetRotation (btQuaternion q);

    void SetColor (float r = 1, float g = 0, float b = 0, float a = 1);

    float GetEmissionRange();
    void SetEmissionRange(float r);


    // ================ Custom Physics ==============================
    //
    // extended calculations for modelling underwater dynamics
//    btVector3 m_gravity;
    
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


    
    /* float height; */
    /* float radius; */

    /* btRigidBody* body; */
    /* std::string meshFilename; */
    
    /* float colr, colg, colb, cola; */
    /* bool drawEmissionRange;     */

    /* int acousticCF; */
    /* int acousticCT; */
    /* DeviceAcousticTransceiver* acoustic; */

    /* aPad (); */
    /* ~aPad (); */

    /* void AddDevices (); */

    /* void Draw (RenderOpenGL* r); */
    /* void Draw (RenderOSG* r); */
    /* void SetColor (float r, float g, float b, float a); */

    /* void Register (PhysicsBullet* p); */
    /* void Unregister (PhysicsBullet* p); */

    /* void Register (RenderOpenGL* r); */
    /* void Unregister (RenderOpenGL* r); */

    /* void SetMeshFilename (std::string filename); */
    /* void Register (RenderOSG* r); */
    /* void Unregister (RenderOSG* r); */

    /* void SetPosition (float x, float y, float z); */
    /* void GetPosition (float& x, float& y, float& z); */

    /* float GetEmissionRange(); */
    /* void SetEmissionRange(float r); */
};

#endif
