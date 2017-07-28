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
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
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

class mhfshape : public btHeightfieldTerrainShape
{
public:

    mhfshape	(
    int 	heightStickWidth,
    int 	heightStickLength,
    const void * 	heightfieldData,
    btScalar 	heightScale,
    btScalar 	minHeight,
    btScalar 	maxHeight,
    int 	upAxis,
    PHY_ScalarType 	heightDataType,
    bool 	flipQuadEdges 
    )		: btHeightfieldTerrainShape
		  (
		      heightStickWidth,
		      heightStickLength,
		      heightfieldData,
		      heightScale,
		      minHeight,
		      maxHeight,
		      upAxis,
		      heightDataType,
		      flipQuadEdges 
		      )
	{
	}

    void 	getVertex (int x, int y, btVector3 &vertex)
	{
	    return btHeightfieldTerrainShape::getVertex(x,y,vertex);
	}
    
    btScalar getRawHeightFieldValue	(	int 	x,
int 	y )
	{
	    return btHeightfieldTerrainShape::getRawHeightFieldValue(x,y);
	}
};


SeaFloor::SeaFloor (float radius, float height, float resolution) :
    Object(),
    PhysicsBulletInterface(),
    RenderOSGInterface()
{
    this->radius = radius;
    this->height = height;
//    this->width = thickness;
    this->borderResolution = resolution;

    // dimGround.setX (radius);
    // dimGround.setY (radius);
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


SeaFloor::~SeaFloor()
{

}

void SeaFloor::setTerrain(string heightFilename, string textureFilename, btVector3 dims)
{
    osg::Image* heightMap = osgDB::readImageFile(heightFilename);
    
    if (renderOSG)
    {
	
	osg::HeightField* heightField = new osg::HeightField();
	
	heightField->allocate(heightMap->s(), heightMap->t());
	heightField->setOrigin(osg::Vec3(-dims.x()/2, -dims.y()/2, 0));
//    heightField->setOrigin(osg::Vec3(-heightMap->s() / 2, -heightMap->t() / 2, 0));
	heightField->setXInterval(dims.x() / heightMap->s());
	heightField->setYInterval(dims.y() / heightMap->t());
	heightField->setSkirtHeight(0.0);
//    heightField->setBorderWidth();
	
	for (int r = 0; r < heightField->getNumRows(); r++)
	{
	    for (int c = 0; c < heightField->getNumColumns(); c++)
	    {
		float height = (*heightMap->data(c, r)) / 255.0f * dims.z();
		heightField->setHeight(c, r, height);
	    }
	}
	
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(new osg::ShapeDrawable(heightField));
	
	osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile(textureFilename));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
		
	RenderOSGInterface::transform->addChild (geode);
    }


    // heightfield in bullet
//    m_rawHeightfieldData = getRawHeightfieldData(m_model, m_type, m_minHeight, m_maxHeight);

    float* data = new float [heightMap->s() * heightMap->t()];
    int i = 0;
    for (int r = 0; r < heightMap->s(); r++)
    {
	for (int c = 0; c < heightMap->t(); c++)
	{
	    float height = (*heightMap->data(c, r)) / 255.0f * dims.z();
	    data[i++] = height;
	}
    }

    
    
    bool flipQuadEdges = false;
    mhfshape * heightfieldShape =
	new mhfshape(heightMap->s(), heightMap->t(),
				      data,
				      dims.z(),
				      0, dims.z(),
				      2, PHY_FLOAT, false);

        // btHeightfieldTerrainShape * heightfieldShape =
	// new btHeightfieldTerrainShape(heightMap->s(), heightMap->t(),
	// 			      data,
	// 			      dims.z(),
	// 			      0, dims.z(),
	// 			      2, PHY_FLOAT, false);

    // scale the shape
//    btVector3 localScaling = getUpVector(m_upAxis, s_gridSpacing, 1.0);
//    heightfieldShape->setLocalScaling(localScaling);
//    btVector3 localScaling = getUpVector(m_upAxis, s_gridSpacing, 1.0);
    float gridSpacing = 0.1;
    heightfieldShape->setLocalScaling(btVector3(dims.x() / heightMap->s(), dims.y() / heightMap->t(), 1.0));
//    heightfieldShape->setLocalScaling(btVector3(0.1, 0.1, 1.0));
    
    // stash this shape away
//    m_collisionShapes.push_back(heightfieldShape);
    
    // set origin to middle of heightfield
    btTransform tr;
    tr.setIdentity();
//    tr.setOrigin(btVector3(-dims.x()/2, -dims.y()/2, dims.z()/2));
    tr.setOrigin(btVector3(0, 0, dims.z()/2));
//    tr.setRotation(btQuaternion(btVector3(0,0,1), M_PI));
//    tr.setOrigin(btVector3(-dims.x()/2, -dims.y()/2, dims.z()/2));
    
    // create ground object

    btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
    btVector3 localInertia(0,0,0);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, myMotionState, heightfieldShape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
//    body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

//    body->setWorldTransform(tr);
    
    // body->setFriction(0.8);
    // body->setHitFraction(0.8);
    // body->setRestitution(0.6);
//    body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
//    body->setActivationState(DISABLE_DEACTIVATION);      
//    PhysicsBulletInterface::physicsBullet->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);



    PhysicsBulletInterface::physicsBullet->m_dynamicsWorld->addRigidBody(body);

    float h = heightfieldShape->getRawHeightFieldValue(512,512);
    std::cout << "HF center " << h << std::endl;
    h = heightfieldShape->getRawHeightFieldValue(700,700);
    std::cout << "HF top right " << h << std::endl;
    h = heightfieldShape->getRawHeightFieldValue(300,300);
    std::cout << "HF bottom left " << h << std::endl;
    h = heightfieldShape->getRawHeightFieldValue(700,300);
    std::cout << "HF top left " << h << std::endl;
    h = heightfieldShape->getRawHeightFieldValue(300,700);
    std::cout << "HF bottom right " << h << std::endl;

    
    btVector3 v;
    int x, y;
    x=512; y=512; heightfieldShape->getVertex (x, y, v);
    std::cout << "HF x,y + vertex " << x << " " << y << " " << " | " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    x=700; y=700; heightfieldShape->getVertex (x, y, v);
    std::cout << "HF x,y + vertex " << x << " " << y << " " << " | " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    x=300; y=300; heightfieldShape->getVertex (x, y, v);
    std::cout << "HF x,y + vertex " << x << " " << y << " " << " | " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    x=0; y=0; heightfieldShape->getVertex (x, y, v);
    std::cout << "HF x,y + vertex " << x << " " << y << " " << " | " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    x=1024; y=1024; heightfieldShape->getVertex (x, y, v);
    std::cout << "HF x,y + vertex " << x << " " << y << " " << " | " << v.x() << " " << v.y() << " " << v.z() << std::endl;

    
	// CREATE A MESH
    // btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
    // btIndexedMesh part;
    
    // part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
    // part.m_vertexStride = sizeof(btScalar) * 3;
    // part.m_numVertices = LandscapeVtxCount[i];
    // part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
    // part.m_triangleIndexStride = sizeof( short) * 3;
    // part.m_numTriangles = LandscapeIdxCount[i]/3;
    // part.m_indexType = PHY_SHORT;
    
    // meshInterface->addIndexedMesh(part,PHY_SHORT);
    
    // bool	useQuantizedAabbCompression = true;
    // btBvhTriangleMeshShape* trimeshShape = new btBvhTriangleMeshShape(meshInterface,useQuantizedAabbCompression);
    // btVector3 localInertia(0,0,0);
    // trans.setOrigin(btVector3(0,-25,0));
    
    // btRigidBody* body = createRigidBody(0,trans,trimeshShape);
    // body->setFriction (btScalar(0.9));
    
}


void SeaFloor::draw (RenderOSG* r)
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


void SeaFloor::registerService(PhysicsBullet* p)
{
    PhysicsBulletInterface::registerService(p);

    // create ground floor
    // btStaticPlaneShape* groundShape = new btStaticPlaneShape (btVector3(0,0,1), 0);
    // p->m_collisionShapes.push_back(groundShape);
    // shapes.push_back(groundShape);
    
    // btTransform groundTransform;
    // groundTransform.setIdentity();
    
    // btScalar mass(0.0);
    // btVector3 localInertia(0,0,0);
    
    // btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    // btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    // btRigidBody* body = new btRigidBody(rbInfo);    
    // body->setUserPointer((void*) static_cast<Object*>(this));
    // p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
    // bodies.push_back(body);

    
    // create the ground floor
    btStaticPlaneShape* groundShape = new btStaticPlaneShape(btVector3(0,0,1), 0);
//    btBoxShape* groundShape = new btBoxShape(dimGround / 2.0);
    p->m_collisionShapes.push_back(groundShape);
    shapes.push_back(groundShape);
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0.0, 0.0, 0.0));
//    groundTransform.setOrigin(btVector3(0.0, 0.0, -dimGround.z() / 2.0));
    
    btScalar mass(0.0);
    btVector3 localInertia(0,0,0);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);    
    body->setUserPointer((void*) static_cast<Object*>(this));
    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
    bodies.push_back(body);

    // create the ceiling
//    btBoxShape* ceilingShape = new btBoxShape(dimGround / 2.0);
    btStaticPlaneShape* ceilingShape = new btStaticPlaneShape(btVector3(0,0,-1), 0);
    p->m_collisionShapes.push_back(ceilingShape);
    shapes.push_back(ceilingShape);
    
    btTransform ceilingTransform;
    ceilingTransform.setIdentity();
    ceilingTransform.setOrigin(btVector3(0.0, 0.0, height));
//    ceilingTransform.setOrigin(btVector3(0.0, 0.0, height + dimGround.z() / 2.0));
        
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
	// btStaticPlaneShape* shape = new btStaticPlaneShape (btVector3(1,0,0), 0);

	// // move the box in its right place...
	// btTransform transform;
	// transform.setIdentity();

	// float x = cos(angle) * (radius + width / 2.0);
	// float y = sin(angle) * (radius + width / 2.0);
	// float z = height / 2.0;

	// transform.setOrigin(btVector3(x, y, z));
	// btQuaternion q(btVector3(0, 0, 1), angle + M_PI / 2.0);
	// transform.setRotation(q);

	
	// p->m_collisionShapes.push_back(shape);
	// shapes.push_back(shape);
	


	
	// find out the correct x size...
	btStaticPlaneShape* shape = new btStaticPlaneShape(btVector3(0,1,0), 0);
//	btBoxShape* shape = new btBoxShape(dimBorder / 2.0);
	p->m_collisionShapes.push_back(shape);
	shapes.push_back(shape);

	// move the box in its right place...
	btTransform transform;
	transform.setIdentity();

	float x = cos(angle) * radius;
	float y = sin(angle) * radius;
	// float x = cos(angle) * (radius + width / 2.0);
	// float y = sin(angle) * (radius + width / 2.0);
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

    // geode->addDrawable(b1d);
    // geode->addDrawable(b2d);
    // geode->addDrawable(b3d);
    // geode->addDrawable(b4d);

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
