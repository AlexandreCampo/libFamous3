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

#include "StaticMesh.h"
#include "Simulator.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

#include <iostream>

#include <osg/TriangleFunctor>

using namespace std;

StaticMesh::StaticMesh() :
    Object(),
    PhysicsBulletInterface(),
    RenderOSGInterface()
{

    colr = 1.0;
    colg = 0.0;
    colb = 0.0;
    cola = 1.0;

    // define collision type
    setCollisionType (1 << 2);
    setCollisionFilter (0x7FFFFFFF);
}
 


StaticMesh::~StaticMesh()
{
}

void StaticMesh::draw (RenderOSG* r)
{
    btScalar ogl[16];
    btTransform t = body->getCenterOfMassTransform();
    t.getOpenGLMatrix( ogl );
    osg::Matrix m(ogl);
    RenderOSGInterface::transform->setMatrix (m);
}

// OSG visitor helper class to extract vertices from 3d mesh
class ShapeBuilder : public osg::NodeVisitor
{
public:

    btBvhTriangleMeshShape* shape = NULL;
    btTriangleMesh* trimesh = NULL;
    
    ShapeBuilder()
	{
	    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	}

    void operator() (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool)
    {
	if (trimesh == NULL) trimesh = new btTriangleMesh();

	trimesh->addTriangle(
	    btVector3(v1.x(), v1.y(), v1.z()),
	    btVector3(v2.x(), v2.y(), v2.z()),
	    btVector3(v3.x(), v3.y(), v3.z()),
	    true);
    }
    
    void apply( osg::Geode& geode )
	{
	    for ( unsigned int i=0; i<geode.getNumDrawables(); ++i )
	    {
		osg::Drawable* drawable = geode.getDrawable(i);
		osg::Geometry* geometry = drawable->asGeometry(); 
		if (geometry) 
		{
		    // instantiate a new copy for gathering triangles...
		    osg::TriangleFunctor<ShapeBuilder> tf;
		    drawable->accept(tf);
		    trimesh = tf.trimesh;
		}		
	    }
	    traverse( geode );
	}

    void apply( osg::Node& node )
	{
	    traverse( node );
	}

    btBvhTriangleMeshShape* getShape()
	{
	    btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(trimesh, true, true);	    
	    return shape;
	}
    
    btTriangleMesh* getTriangleMesh()
	{
	    return trimesh;
	}
};



void StaticMesh::registerService (PhysicsBullet* p)
{
    PhysicsBulletInterface::registerService(p);
    
    setTimestep(p->getTimestep());

    // use OSG to load the mesh file
    if (meshFilename.empty())
    {
	cout << "No 3d model defined for BulletPhysics/OSG Render..." << endl;
	return;
    }

    if (!node) node = osgDB::readNodeFile(meshFilename);
    
    if (!node)
    {
        std::cout << "Could not load triangleMesh model." << std::endl;
    }    

    // instantiate helper class that will be called by OSG
    osg::TriangleFunctor<ShapeBuilder> shapeBuilder;

    // ask OSG to explore the mesh
    node->accept(shapeBuilder);

    // extract shape for bullet physics
    shape = shapeBuilder.getShape();
    triangleMesh = shapeBuilder.getTriangleMesh();
    
    principalTransform.setIdentity();
    principalTransformInverse = principalTransform.inverse();

    // create body
    btDefaultMotionState* myMotionState = new btDefaultMotionState(principalTransform);
    btVector3 localInertia(0,0,0);
    float mass = 0.0;
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
    
    rbInfo.m_friction = 0.8;    

    body = new btRigidBody(rbInfo);
    body->setUserPointer((void*) static_cast<Object*>(this));
    body->setActivationState(DISABLE_DEACTIVATION);
 
    p->m_collisionShapes.push_back(shape);
    p->m_dynamicsWorld->addRigidBody(body);
}

void StaticMesh::unregisterService (PhysicsBullet* p)
{
}


void StaticMesh::setMeshFilename (std::string filename)
{
    meshFilename = filename;
}

void StaticMesh::registerService (RenderOSG* r)
{
    RenderOSGInterface::registerService (r);

    if (meshFilename.empty())
    {
	cout << "No 3d model defined for OSG Render..." << endl;
	return;
    }

    if (!node) node = osgDB::readNodeFile(meshFilename);

    if (!node)
    {
        std::cout << "Could not load triangleMesh model." << std::endl;
    }    

    // one transform for spatial position, will be adjuted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();
    
    // one transform for scaling model
    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
    t->setScale(osg::Vec3(1, 1, 1));

    RenderOSGInterface::transform->addChild (t);
    t->addChild (node);
    r->root->addChild(RenderOSGInterface::transform);

    // text overlay for robot
    btVector3 aabbMin = ((btBvhTriangleMeshShape*)shape)->getLocalAabbMin();
    btVector3 aabbMax = ((btBvhTriangleMeshShape*)shape)->getLocalAabbMax();
    osg::Vec3 pos (
	aabbMin.x(),
	(aabbMax.y() - aabbMin.y()) / 2,
	aabbMax.z());
    text = r->createText(pos, "", (aabbMax.x() - aabbMin.x()) / 2);
    textGeode = new osg::Geode;
    textGeode->addDrawable( text );
    textGeode->setNodeMask(0);

    RenderOSGInterface::transform->addChild (textGeode);
}

void StaticMesh::setTextDrawable(bool d)
{
    if (renderOSG)
    {
	textGeode->setNodeMask(d);
    }
}
    
void StaticMesh::setText(string s)
{
    if (renderOSG)
    {
	text->setText(s);
    }
}

void StaticMesh::setTextColor(float r, float g, float b, float a)
{
    if (renderOSG)
    {
	text->setColor(osg::Vec4(r, g, b, a));
    }
}
    

void StaticMesh::unregisterService (RenderOSG* r)
{
    RenderOSGInterface::unregisterService(r);
}

void StaticMesh::setPosition (btVector3 p)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(p);
    body->setCenterOfMassTransform(t);
}

btVector3 StaticMesh::getPosition ()
{
    return body->getCenterOfMassTransform().getOrigin();
}


void StaticMesh::setRotation (btQuaternion q)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(q);
    body->setCenterOfMassTransform(t);
}

btQuaternion StaticMesh::getRotation (btQuaternion q)
{
    return body->getOrientation();
}


void StaticMesh::setColor (float r, float g, float b, float a)
{
    this->colr = r;
    this->colg = g;
    this->colb = b;
    this->cola = a;

    if (renderOSG)
    {
	osg::ref_ptr<osg::Material> mat = new osg::Material;
	mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(colr, colg, colb, cola));
	RenderOSGInterface::transform->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }
}

