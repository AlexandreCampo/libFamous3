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

#ifndef SEA_FLOOR_H
#define SEA_FLOOR_H

#include "Object.h"
#include "RenderOpenGLInterface.h"
#include "RenderOSGInterface.h"
#include "PhysicsBulletInterface.h"
#include "WaterVolumeInterface.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include <vector>

class SeaFloor : public virtual Object, public PhysicsBulletInterface, public RenderOSGInterface, public WaterVolumeInterface
{
public:

    float radius;
    float height;
//    float width;
    float borderResolution;
//    btVector3 dimGround;
//    btVector3 dimBorder;

    int displayList;
    std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> cubeTransforms;

    osg::ref_ptr<osg::PositionAttitudeTransform> terrainTransform;

    // the aquarium is made of several boxes (ground, then walls around)
    std::vector<btCollisionShape*> shapes;
    std::vector<btRigidBody*> bodies;

    SeaFloor(float radius = 3.0, float height = 2.0, float resolution = 40.0);
    ~SeaFloor();

    void draw (RenderOSG* r);

    void registerService (PhysicsBullet* p);
    void unregisterService (PhysicsBullet* p);

    void registerService (RenderOSG* r);
    void unregisterService (RenderOSG* r);

    void registerService (WaterVolume* r);
    void unregisterService (WaterVolume* r);

    void setTerrain(std::string heightFilename, std::string textureFilename, btVector3 dims);
    
};

#endif
