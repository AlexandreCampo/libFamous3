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

#ifndef AQUARIUM_CIRCULAR_H
#define AQUARIUM_CIRCULAR_H

#include "Object.h"
#include "RenderOpenGLInterface.h"
#include "RenderOSGInterface.h"
#include "PhysicsBulletInterface.h"
#include "WaterVolumeInterface.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include <vector>

class AquariumCircular : public virtual Object, public RenderOpenGLInterface, public PhysicsBulletInterface, public RenderOSGInterface, public WaterVolumeInterface
{
public:

    float radius;
    float height;
    float width;
    float borderResolution;
    btVector3 dimGround;
    btVector3 dimBorder;

    int displayList;
    std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> cubeTransforms;

    // the aquarium is made of several boxes (ground, then walls around)
    std::vector<btCollisionShape*> shapes;
    std::vector<btRigidBody*> bodies;

    AquariumCircular(float radius = 3.0, float height = 2.0, float thickness = 1.0, float resolution = 40.0);
    ~AquariumCircular();

    void draw (RenderOpenGL* r);
    void draw (RenderOSG* r);
    void buildDisplayList ();

    void registerService (PhysicsBullet* p);
    void unregisterService (PhysicsBullet* p);

    void registerService (RenderOpenGL* r);
    void unregisterService (RenderOpenGL* r);

    void registerService (RenderOSG* r);
    void unregisterService (RenderOSG* r);

    void registerService (WaterVolume* r);
    void unregisterService (WaterVolume* r);
};

#endif
