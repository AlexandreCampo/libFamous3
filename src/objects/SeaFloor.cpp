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

#include "SeaFloor.h"
#include "Simulator.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btDefaultMotionState.h"

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Material>

#include <iostream>
#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  

using namespace std;


SeaFloor::SeaFloor (float radius, float height, float resolution) :
    Object(),
    PhysicsBulletInterface(),
    RenderOSGInterface()
{
    this->radius = radius;
    this->height = height;
    this->borderResolution = resolution;

    // define collision type
    setCollisionType (1<<1);
    setCollisionFilter (0x7FFFFFFF);
}


SeaFloor::~SeaFloor()
{

}

void SeaFloor::setTerrain(string heightFilename, string textureFilename, btVector3 dims)
{
    // first, free any previously allocated heightField shape / nodes
    if (heightFieldBody)
    {
	delete[] heightFieldData;
	
	PhysicsBulletInterface::physicsBullet->m_dynamicsWorld->removeRigidBody(heightFieldBody);
	delete heightFieldBody->getMotionState();
	delete heightFieldBody;
	delete heightFieldShape;

	heightFieldShape = NULL;
	heightFieldBody = NULL;

	heightFieldGeode = NULL;
	heightFieldDrawable = NULL;
	heightFieldTexture = NULL;
    }

    
    // load the height map
    osg::Image* heightMap = osgDB::readImageFile(heightFilename);

    heightFieldData = new float [heightMap->s() * heightMap->t()];
    int i = 0;
    for (int r = 0; r < heightMap->s(); r++)
    {
	for (int c = 0; c < heightMap->t(); c++)
	{
	    float height = (*heightMap->data(c, r)) / 255.0f * dims.z();
	    heightFieldData[i++] = height;
	}
    }
    
    // heightField in bullet      
    bool flipQuadEdges = false;
    heightFieldShape = 	new btHeightfieldTerrainShape(heightMap->s(), heightMap->t(),
						      heightFieldData,
						      dims.z(),
						      0, dims.z(),
						      2, PHY_FLOAT, false);

    heightFieldShape->setLocalScaling(btVector3(dims.x() / heightMap->s(), dims.y() / heightMap->t(), 1.0));
    
    // set origin to middle of heightField
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, 0, dims.z()/2));

    // create ground object
    btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
    btVector3 localInertia(0,0,0);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, myMotionState, heightFieldShape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
    PhysicsBulletInterface::physicsBullet->m_dynamicsWorld->addRigidBody(body);

    // add to render if present
    if (renderOSG)
    {	
	osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField();
	
	heightField->allocate(heightMap->s(), heightMap->t());
	heightField->setOrigin(osg::Vec3(-dims.x()/2, -dims.y()/2, 0));
	heightField->setXInterval(dims.x() / heightMap->s());
	heightField->setYInterval(dims.y() / heightMap->t());
	heightField->setSkirtHeight(0.0);
	
	for (int r = 0; r < heightField->getNumRows(); r++)
	{
	    for (int c = 0; c < heightField->getNumColumns(); c++)
	    {
		float height = (*heightMap->data(c, r)) / 255.0f * dims.z();
		heightField->setHeight(c, r, height);
	    }
	}
	
	heightFieldGeode = new osg::Geode();
	heightFieldDrawable = new osg::ShapeDrawable(heightField);
	heightFieldGeode->addDrawable(heightFieldDrawable);
	
	heightFieldTexture = new osg::Texture2D(osgDB::readImageFile(textureFilename));
	heightFieldTexture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	heightFieldTexture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	heightFieldTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	heightFieldTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	heightFieldGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, heightFieldTexture);
		
	RenderOSGInterface::transform->addChild (heightFieldGeode);
    }
}


void SeaFloor::draw (RenderOSG* r)
{
    // update position of cubes that act as water surface
    for (auto t : cubeTransforms)
    {
	osg::Vec3d pos = t->getPosition();
	float height = waterVolume->getHeight(btVector3(pos.x(), pos.y(), 0), simulator->time);
	pos.set(pos.x(), pos.y(), height);
	t->setPosition(pos);

	osg::Vec3d p = t->getPosition();
    }
}


void SeaFloor::registerService(PhysicsBullet* p)
{
    PhysicsBulletInterface::registerService(p);
    
    // create the ground floor
    btStaticPlaneShape* groundShape = new btStaticPlaneShape(btVector3(0,0,1), 0);
    p->m_collisionShapes.push_back(groundShape);
    shapes.push_back(groundShape);
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0.0, 0.0, 0.0));
    
    btScalar mass(0.0);
    btVector3 localInertia(0,0,0);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);    
    body->setUserPointer((void*) static_cast<Object*>(this));
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
    bodies.push_back(body);

    // create the ceiling
    btStaticPlaneShape* ceilingShape = new btStaticPlaneShape(btVector3(0,0,-1), 0);
    p->m_collisionShapes.push_back(ceilingShape);
    shapes.push_back(ceilingShape);
    
    btTransform ceilingTransform;
    ceilingTransform.setIdentity();
    ceilingTransform.setOrigin(btVector3(0.0, 0.0, height));
        
    btDefaultMotionState* myMotionState2 = new btDefaultMotionState(ceilingTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo2(mass,myMotionState2,ceilingShape,localInertia);
    rbInfo2.m_friction = 0.0;
    btRigidBody* body2 = new btRigidBody(rbInfo2);    
    body2->setUserPointer((void*) static_cast<Object*>(this));
    p->m_dynamicsWorld->addRigidBody(body2, collisionType, collisionFilter);
    bodies.push_back(body2);

    // now create the aquarium as a set of boxes
    float angle = 0.0;	
    float angleStep = 2.0 * M_PI / borderResolution;
    for (int i = 0; i < (int)borderResolution; i++)
    {	
	// find out the correct x size...
	btStaticPlaneShape* shape = new btStaticPlaneShape(btVector3(0,1,0), 0);
	p->m_collisionShapes.push_back(shape);
	shapes.push_back(shape);

	// move the box in its right place...
	btTransform transform;
	transform.setIdentity();

	float x = cos(angle) * radius;
	float y = sin(angle) * radius;
	float z = height / 2.0;

	transform.setOrigin(btVector3(x, y, z));
	btQuaternion q(btVector3(0, 0, 1), angle + M_PI / 2.0);
	transform.setRotation(q);
	
	btScalar mass(0.0);
	btVector3 localInertia(0,0,0);
    
	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* b = new btRigidBody(rbInfo);    
	b->setUserPointer((void*) static_cast<Object*>(this));
	p->m_dynamicsWorld->addRigidBody(b, collisionType, collisionFilter);
	bodies.push_back(b);

	angle += angleStep;
    }
}

void SeaFloor::unregisterService(PhysicsBullet* p)
{
    
}


void SeaFloor::registerService (RenderOSG* r)
{
    RenderOSGInterface::registerService(r);

    osg::Geode* geode = new osg::Geode();
 
    osg::ref_ptr<osg::Vec4Array> shared_colors = new osg::Vec4Array;
    shared_colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // same trick for shared normal.
    osg::ref_ptr<osg::Vec3Array> shared_normals = new osg::Vec3Array;
    shared_normals->push_back(osg::Vec3(0.0f,1.0f,0.0f));
      
    float l = height;
    int i;
    float tmp,ny,nz,a,ca,sa;
    const int n = borderResolution;	// number of sides to the cylinder (divisible by 4)
    
    a = float(M_PI*2.0)/float(n);
    sa = (float) sin(a);
    ca = (float) cos(a);
    
    int ringsCount = 3;
    float zstep = l / float(ringsCount);
    float zoffset = l + 0.05;
    for (int zs = 0; zs <= ringsCount; zs++)
    {
	// draw cylinder body
	osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry;            
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

	ny=1; nz=0;		  // normal vector = (0,ny,nz)
	for (i=0; i<=n; i++) 
	{
	    vertices->push_back(osg::Vec3(ny*radius, nz*radius, zoffset));
	    
	    // rotate ny,nz
	    tmp = ca*ny - sa*nz;
	    nz = sa*ny + ca*nz;
	    ny = tmp;
	}
	polyGeom->setVertexArray(vertices);        
	polyGeom->setColorArray(shared_colors);
	polyGeom->setColorBinding(osg::Geometry::BIND_OVERALL);       
	polyGeom->setNormalArray(shared_normals);
	polyGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);        
	polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices->size()));
	polyGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->addDrawable(polyGeom);

	zoffset -= zstep;
    }
    
    node = geode;
    transform = new osg::MatrixTransform();
    transform->addChild (node);
    r->root->addChild(transform);


    osg::Geode* geode2 = new osg::Geode();

    osg::ref_ptr<osg::Box> b = new osg::Box(osg::Vec3(0,0,0), radius/100.0, radius/100.0, radius/100.0);
    osg::ref_ptr<osg::ShapeDrawable> bd = new osg::ShapeDrawable(b);

    float cr = 100.0 / 254.0;
    float cg = 100.0 / 254.0;
    float cb = 254.0 / 254.0;
    osg::ref_ptr<osg::Material> material3 = new osg::Material();
    material3->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
    material3->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(cr * 0.1, cg * 0.1, cb * 0.1, 1.0));
    material3->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
    material3->setShininess(osg::Material::FRONT_AND_BACK, 3.0);

    bd->getOrCreateStateSet()->setAttribute(material3);

    geode2->addDrawable(bd);
    
    // for (auto* t : cubeTransforms) delete t;
    cubeTransforms.clear();

    for (float x = -radius; x < radius; x += 0.25)
    {
	for (float y = -radius; y < radius; y += 0.25)
	{
	    // skip position out of circle
	    if (x * x + y * y > radius * radius) continue;
	    
	    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
	    osg::Vec3d pos (x, y, 0);
	    osg::Quat q(1,1,1,0);
	    t->setAttitude(q);
	    t->setPosition(pos);
	    t->addChild(geode2);	    
	    r->root->addChild(t);
	    cubeTransforms.push_back(t);
	}
    }    
}

void SeaFloor::unregisterService (RenderOSG* r)
{
    RenderOSGInterface::unregisterService(r);
}

void SeaFloor::registerService (WaterVolume* w)
{
    WaterVolumeInterface::registerService(w);
}

void SeaFloor::unregisterService (WaterVolume* w)
{
    WaterVolumeInterface::unregisterService(w);
}
