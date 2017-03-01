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

#include "DevicePropeller.h"

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

#include <iostream>



DevicePropeller::DevicePropeller(PhysicsBullet* p, btRigidBody* body, float maxForce) 
{
    SetActionTimeStep(p->GetTimeStep());
    
    this->body = body;    
    this->maxForce = maxForce;

    this->transform.setIdentity();
    
    SetSpeed(0);
    drawable = false;
}


DevicePropeller::~DevicePropeller()
{

}

void DevicePropeller::SetPosition(btVector3 position)
{
    this->transform.setOrigin(position);
}

btVector3 DevicePropeller::GetPosition()
{
    return transform.getOrigin();
}

void DevicePropeller::SetOrientation(btQuaternion q)
{
    transform.setRotation(q);
}

btQuaternion DevicePropeller::GetOrientation()
{
    return transform.getRotation();
}

void DevicePropeller::SetSpeed(float s)
{
    speed = s;
    force = maxForce * s;
}

float DevicePropeller::GetSpeed()
{
    return speed;
}

void DevicePropeller::SetDrawable(bool d)
{
    drawable = d;
}

bool DevicePropeller::IsDrawable()
{
    return drawable;
}

void DevicePropeller::ActionStep()
{
    const btMatrix3x3& bodyRotation = body->getCenterOfMassTransform().getBasis();

    btVector3 f = bodyRotation * transform.getBasis() * btVector3(force, 0, 0);
    btVector3 relpos = bodyRotation * transform.getOrigin();
    btVector3 t = relpos.cross(f * body->getLinearFactor());
    
    body->applyCentralForce(f);
    body->applyTorque(t);
}

void DevicePropeller::PerceptionStep()
{
}

void DevicePropeller::Reset()
{
    force = 0;
}

void DevicePropeller::Register (RenderOSG* r)
{
    RenderOSGInterface::Register (r);

    if (!devicePropellerNode)
    {
    	osg::ref_ptr<osg::Geode> geode; 
    	osg::ref_ptr<osg::Cylinder> cylinder; 
    	osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable;
	
    	cylinder = new osg::Cylinder(osg::Vec3(0,0,0),0.01,0.1); 
    	cylinderDrawable = new osg::ShapeDrawable(cylinder ); 
    	geode = new osg::Geode; 
    	geode->addDrawable(cylinderDrawable); 
    	devicePropellerNode = geode;
    }
    
    // one transform for spatial position, will be adjusted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();

    // rotate, translate, scale to device location
    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
    btVector3 p = transform.getOrigin();
    t->setPosition(osg::Vec3d(p.getX(), p.getY(), p.getZ()));
    t->setAttitude(osg::Quat( M_PI/2.0, osg::Vec3(0,1,0) ));

    RenderOSGInterface::transform->addChild (t);
    t->addChild (devicePropellerNode);
    r->root->addChild(RenderOSGInterface::transform);
}

void DevicePropeller::Unregister (RenderOSG* r)
{
    RenderOSGInterface::Unregister(r);
}

void DevicePropeller::Draw (RenderOSG* r)
{
    btScalar ogl[16];
    btTransform t = body->getCenterOfMassTransform();
    t.getOpenGLMatrix( ogl );
    osg::Matrix m(ogl);
    RenderOSGInterface::transform->setMatrix (m);

    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 0.5));
    devicePropellerNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);    
}
