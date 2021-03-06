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

#include "AquariumCircular.h"
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

AquariumCircular::AquariumCircular (float radius, float height, float resolution) :
    Object(),
    PhysicsBulletInterface(),
    RenderOSGInterface()
{
    this->radius = radius;
    this->height = height;

    // thickness is an obsolete parameter... not used anymore.
//    this->width = thickness;
    this->borderResolution = resolution;

    // dimGround.setX (radius * 2.0 + width);
    // dimGround.setY (radius * 2.0 + width);
    // dimGround.setZ (width);

    // border component length is related to border perimeter
    // float perimeter = 2.0 * radius * M_PI;
    // float angle = 2.0 * M_PI / borderResolution;
    // dimBorder.setX (tan(angle / 2.0) * radius * 3.0);
    // dimBorder.setY (width);
    // dimBorder.setZ (height);

    // define collision type
    setCollisionType (1<<1);
    setCollisionFilter (0x7FFFFFFF);
}


AquariumCircular::~AquariumCircular()
{

}


void AquariumCircular::draw (RenderOSG* r)
{
    // btScalar ogl[16];
    // btTransform t = body->getCenterOfMassTransform() * principalTransform.inverse();
    // t.getOpenGLMatrix( ogl );
    // osg::Matrix m(ogl);
    // transform->setMatrix (m);

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


void AquariumCircular::registerService(PhysicsBullet* p)
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

void AquariumCircular::unregisterService(PhysicsBullet* p)
{
    
}

void AquariumCircular::registerService (RenderOSG* r)
{
    RenderOSGInterface::registerService(r);

    osg::Geode* geode = new osg::Geode();
 
    osg::ref_ptr<osg::Vec4Array> shared_colors = new osg::Vec4Array;
    shared_colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // same trick for shared normal.
    osg::ref_ptr<osg::Vec3Array> shared_normals = new osg::Vec3Array;
    shared_normals->push_back(osg::Vec3(0.0f,1.0f,0.0f));
      
    float l = height;
//    float rad = radius;
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
//	glBegin (GL_LINE_STRIP);
	for (i=0; i<=n; i++) 
	{
	    vertices->push_back(osg::Vec3(ny*radius, nz*radius, zoffset));
//	    normals.push_back(Vec3(ny*r,nz*r,zoffset));
	    // glNormal3d (ny,nz,zoffset);
	    // glVertex3d (ny*r,nz*r,zoffset);
	    
	    // rotate ny,nz
	    tmp = ca*ny - sa*nz;
	    nz = sa*ny + ca*nz;
	    ny = tmp;
	}
//	glEnd();
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
    
    // add walls
    float wallWidth = 0.07;
    osg::ref_ptr<osg::Box> b1 = new osg::Box(osg::Vec3(0,-radius - wallWidth / 2.0, height/2), radius * 2, wallWidth, height);
    osg::ref_ptr<osg::ShapeDrawable> b1d = new osg::ShapeDrawable(b1);

    osg::ref_ptr<osg::Box> b2 = new osg::Box(osg::Vec3(0,radius + wallWidth / 2.0, height/2), radius * 2, wallWidth, height);
    osg::ref_ptr<osg::ShapeDrawable> b2d = new osg::ShapeDrawable(b2);

    osg::ref_ptr<osg::Box> b3 = new osg::Box(osg::Vec3(-radius - wallWidth / 2.0, 0, height/2), wallWidth, radius * 2 + wallWidth*2, height);
    osg::ref_ptr<osg::ShapeDrawable> b3d = new osg::ShapeDrawable(b3);

    osg::ref_ptr<osg::Box> b4 = new osg::Box(osg::Vec3(-wallWidth/2, 0, -wallWidth/2), radius*2 + wallWidth, radius*2 + wallWidth*2, wallWidth);
    osg::ref_ptr<osg::ShapeDrawable> b4d = new osg::ShapeDrawable(b4);


    float cr = 52.0 / 254.0;
    float cg = 93.0 / 254.0;
    float cb = 169.0 / 254.0;
    osg::ref_ptr<osg::Material> material = new osg::Material();
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(cr * 0.1, cg * 0.1, cb * 0.1, 1.0));
    material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
//    material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(52.0 / 254.0, 93.0 / 254.0, 169.0 / 254.0, 0.3));    
    material->setShininess(osg::Material::FRONT_AND_BACK, 3.0);
//    material->setColorMode (osg::Material::DIFFUSE);
    
// assign the material to the sphere and cube
    b1d->getOrCreateStateSet()->setAttribute(material);
    b2d->getOrCreateStateSet()->setAttribute(material);
    b3d->getOrCreateStateSet()->setAttribute(material);

    cr = 122.0 / 254.0;
    cg = 85.0 / 254.0;
    cb = 56.0 / 254.0;
    osg::ref_ptr<osg::Material> material2 = new osg::Material();
    material2->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
    material2->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(cr * 0.1, cg * 0.1, cb * 0.1, 1.0));
    material2->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(cr, cg, cb, 1.0));
//    material2->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(52.0 / 254.0, 93.0 / 254.0, 169.0 / 254.0, 0.3));    
    material2->setShininess(osg::Material::FRONT_AND_BACK, 3.0);
    b4d->getOrCreateStateSet()->setAttribute(material2);

    // b1d->setColor (osg::Vec4(52.0 / 254.0, 93.0 / 254.0, 169.0 / 254.0, 1));
    // b2d->setColor (osg::Vec4(52.0 / 254.0, 93.0 / 254.0, 169.0 / 254.0, 1));
    // b3d->setColor (osg::Vec4(52.0 / 254.0, 93.0 / 254.0, 169.0 / 254.0, 1));
    // b4d->setColor (osg::Vec4(122.0 / 254.0, 85.0 / 254.0, 56.0 / 254.0, 1));

    geode->addDrawable(b1d);
    geode->addDrawable(b2d);
    geode->addDrawable(b3d);
    geode->addDrawable(b4d);

    node = geode;
    transform = new osg::MatrixTransform();
    transform->addChild (node);
    r->root->addChild(transform);


    osg::Geode* geode2 = new osg::Geode();

    osg::ref_ptr<osg::Box> b = new osg::Box(osg::Vec3(0,0,0), radius/100.0, radius/100.0, radius/100.0);
    osg::ref_ptr<osg::ShapeDrawable> bd = new osg::ShapeDrawable(b);

    cr = 100.0 / 254.0;
    cg = 100.0 / 254.0;
    cb = 254.0 / 254.0;
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

void AquariumCircular::unregisterService (RenderOSG* r)
{
    RenderOSGInterface::unregisterService(r);
}

void AquariumCircular::registerService (WaterVolume* w)
{
    WaterVolumeInterface::registerService(w);
}

void AquariumCircular::unregisterService (WaterVolume* w)
{
    WaterVolumeInterface::unregisterService(w);
}
