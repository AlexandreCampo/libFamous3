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

#ifndef STATIC_MESH_H
#define STATIC_MESH_H

#include "Object.h"
#include "RenderOSGInterface.h"
#include "PhysicsBulletInterface.h"

class StaticMesh : public virtual Object, public PhysicsBulletInterface, public RenderOSGInterface
{
public:
        
    btCollisionShape* shape;
    btRigidBody* body;

    btTriangleMesh* triangleMesh;
    osg::ref_ptr<osg::Node> node = NULL;
    
    btTransform principalTransform;
    btTransform principalTransformInverse;
//    btVector3 centerOfVolume;
            
    std::string meshFilename;
    osgText::Text* text;
    osg::ref_ptr<osg::Geode> textGeode;
        
    float colr, colg, colb, cola;

    StaticMesh ();
    ~StaticMesh ();

    void draw (RenderOSG* r);

    void registerService (PhysicsBullet* p);
    void unregisterService (PhysicsBullet* p);

    void setMeshFilename (std::string filename);
    void registerService (RenderOSG* r);
    void unregisterService (RenderOSG* r);

    void setPosition (btVector3 p);
    btVector3 getPosition ();
    void setRotation (btQuaternion q);
    btQuaternion getRotation (btQuaternion q);

    void setColor (float r = 1, float g = 0, float b = 0, float a = 1);

    void setTextDrawable(bool d);
    void setText(std::string s);
    void setTextColor(float r, float g, float b, float a = 1);
    
};

#endif
