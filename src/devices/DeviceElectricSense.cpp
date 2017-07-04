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

#include "DeviceElectricSense.h"

//#include "PhysicsBullet.h"
//#include "PhysicsBulletInterface.h"
//#include "RenderOpenGLInterface.h"
//#include "RenderOSGInterface.h"
#include "Object.h"

//#include <list>
//#include <vector>
#include <iostream>
#include <typeinfo>
//#include <eigen3>
//#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

DeviceElectricSense::DeviceElectricSense(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType, float maxRange) : 
    btBroadphaseAabbCallback (),
    RenderOpenGLInterface()
{
    this->physics = p;
    this->parentBody = b;
    this->localTransform = t;

    // inherited from ContactResultCallback
    this->collisionFilter = collisionFilter;
    this->collisionType = collisionType;

    // create the collision body
    this->maxRange = maxRange;
    collisionShape = new btSphereShape(maxRange);
    
    btScalar mass(0.0);
    btVector3 localInertia(0.0, 0.0, 0.0);
 		
    btTransform startTransform;
    startTransform.setIdentity();
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,collisionShape,localInertia);
    collisionBody = new btRigidBody(rbInfo);    

    // make sure parent body has a user pointer correctly set... 
    // I expect this convention to cause troubles...
    // the user pointer should be set to the corresponding Object* pointer (see objects implemenations such as Floor.cpp)
    if (parentBody->getUserPointer() == NULL)
    {
	std::cerr << "The body " << parentBody << " misses a proper user pointer in its collision shape !" << std::endl;
    }

    reset();
}


DeviceElectricSense::~DeviceElectricSense()
{

}

void DeviceElectricSense::actionStep ()
{
}

void DeviceElectricSense::perceptionStep ()
{
    // position of electrodes not yet expressed in global frame
    transformed = false;
    inducedICalculated = false;
}
    
void DeviceElectricSense::reset ()
{
}

void DeviceElectricSense::draw (RenderOSG* r)
{   
}

void DeviceElectricSense::draw (RenderOpenGL* r)
{   
    btTransform transform;
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();

    transform = parentBody->getWorldTransform();
    
    transform *= localTransform;
    transform.getOpenGLMatrix(m);

    glPushMatrix();     
    glMultMatrixf(m);


    dsRotationMatrix rotmat;
    float pos[3]; pos[0] = 0; pos[1] = 0; pos[2] = 0;       
    dRotation2dsRotationMatrix(0, rotmat);
    r->dsSetColorAlpha (0.9, 0.9, 0.9, 0.3);
    r->dsDrawCylinder((float*) &pos, (float*) &rotmat, 0.1, maxRange);

    glPopMatrix();
}

void DeviceElectricSense::send (int content)
{
}

bool DeviceElectricSense::receive (int& content)
{
}

void DeviceElectricSense::addElectrode(btVector3 position)
{
    electrodePositions.push_back(position);
    electrodePositionsTransformed.push_back(position);

    // redim vectors and mats
    numElectrodes = electrodePositions.size();

    C0 = MatrixXf(numElectrodes,numElectrodes);
    I0 = VectorXf(numElectrodes);
    I = VectorXf(numElectrodes);

    polarization = VectorXf(numElectrodes);
//    inducedI = VectorXf(numElectrodes);    
}

void DeviceElectricSense::setPolarization (VectorXf& polarization)
{
    this->polarization = polarization;
    I0 = C0 * polarization;

    // std::cout << "set pola+I0 "
    // 	      << polarization(0) << " "
    // 	      << polarization(1) << " "
    // 	      << polarization(2) << " "
    // 	      << polarization(3) << " "
    // 	      << polarization(4) << " "
    // 	      << " | "
    // 	      << I0(0) << " "
    // 	      << I0(1) << " "
    // 	      << I0(2) << " "
    // 	      << I0(3) << " "
    // 	      << I0(4) << " " << std::endl;

}

void DeviceElectricSense::setC0 (MatrixXf& C0)
{
    this->C0 = C0;
}

bool DeviceElectricSense::process (const btBroadphaseProxy *proxy)
{
    // do not take into account oneself, or parent
    btCollisionObject* detectedObject = (btCollisionObject*)proxy->m_clientObject;
    if (detectedObject == collisionBody) return true;
    if (detectedObject == parentBody) return true;
	
    //only perform raycast if filterMask matches
    btBroadphaseProxy* bph = detectedObject->getBroadphaseHandle();
    bool collides = (bph->m_collisionFilterGroup & this->collisionFilter) != 0;
    collides = collides && (this->collisionType & bph->m_collisionFilterMask);

    if (collides) 
    {
	// We say the signal is perceived iff the object COM falls within the emission radius...
	btTransform other = detectedObject->getWorldTransform();
	btTransform self = collisionBody->getWorldTransform();	

        btVector3 diff = other.getOrigin() - self.getOrigin();
        btScalar dist = diff.length();

        if ( dist > maxRange)
        {
	    return true;
        }

	// ok if the object has corresponding sensor, it detects the signal
	Object* o = (Object*) detectedObject->getUserPointer();
	
	for (unsigned int i = 0; i < o->devices.size(); i++)
	{
	    // found the device	
	    if (typeid (*(o->devices[i])) == typeid(*this))
	    {
		DeviceElectricSense* od = (DeviceElectricSense*) o->devices[i];
		
		// build the lists of nearby devices, categorized as active or passive
		if (od->polarized)
		{
		    activeDevices.push_back(od);
		}		    
		else if (!od->polarized && dist < maxRange / 3.0)
		{
		    passiveDevices.push_back(od);
		}

		return true;
	    }
	}
    }
    return true;
}

VectorXf DeviceElectricSense::getCurrents()
{
    // move the sensor in its place with local transform
    btTransform tr = parentBody->getWorldTransform() * localTransform;
    collisionBody->setWorldTransform(tr);
    
    // use the engine to make half work
    btVector3 aabbMin,aabbMax;
    collisionBody->getCollisionShape()->getAabb(tr, aabbMin, aabbMax);

    // clear list of neighboring devices
    activeDevices.clear();
    passiveDevices.clear();
    
    // process will be called when a collision is detected
    btBroadphaseInterface* broadphase =  physics->m_dynamicsWorld->getBroadphase();
    broadphase->aabbTest(aabbMin,aabbMax, (*this));

    std::cout << "Dev " << this << " found " << activeDevices.size() << " active and " << passiveDevices.size() << " passive | ";
    
    
    // init vector of measured currents
    I = VectorXf::Zero(numElectrodes);
            
    // add current contributions from active neighbours
    if (inducedICalculated)
    {
	I += inducedI;

	std::cout << " added precalc induced " << inducedI(0) << " " << inducedI(1) << " " << inducedI(2) << " " << inducedI(3) << " " << inducedI(4) << std::endl;
    }
    else
    {	
	for (auto* od : activeDevices)
	{
	    addContribution(od, this, od->I0, I);
	}
	inducedI = I;
	inducedICalculated = true;
    }

    // at least one device must be active
    if (activeDevices.empty()) return I;
  
    // add self to polarized devices
    if (polarized) activeDevices.push_back(this);
    
    // add contribution from passive neighbours with currents induced by surrounding active devices
    for (auto* od : passiveDevices)
    {
	// calculate induced currents first
	if (!od->inducedICalculated)
	{
	    od->inducedI = VectorXf::Zero(od->numElectrodes);
	    for (auto* oda : activeDevices)
	    {            
		addContribution(oda, od, oda->I0, od->inducedI);
	    }
	    od->inducedICalculated = true;
	}

	// add current from passive devices to self
	I += od->inducedI;
    }
    
    // wrap up...
    for(int i = 0; i < numElectrodes; i++)
    {
        if (abs( I(i) ) > abs( I0(i) ) )
        {
	    if (I(i) < 0.0)		
		I(i) = - abs( I0(i) );
	    else
		I(i) = abs( I0(i) );
        }
    }
    
    return I;
}

void DeviceElectricSense::addContribution(DeviceElectricSense* src, DeviceElectricSense* dst, VectorXf& ISrc, VectorXf& IDst)
{
    // transform electrode pos from local to global mark
    if (!src->transformed)
    {	
	const btTransform& tr = src->parentBody->getWorldTransform() * src->localTransform;
	for (unsigned int i = 0; i < src->numElectrodes; ++i)
	{
	    src->electrodePositionsTransformed[i] = tr * electrodePositions[i];
	    src->transformed = true;
	}
    }
    if (!dst->transformed)
    {	
	const btTransform& tr = dst->parentBody->getWorldTransform() * dst->localTransform;
	for (unsigned int i = 0; i < dst->numElectrodes; ++i)
	{
	    dst->electrodePositionsTransformed[i] = tr * electrodePositions[i];
	    dst->transformed = true;
	}
    }	

    // calculate contributions from each src electrode to all dst electrodes
    VectorXf contributions = VectorXf::Zero(dst->numElectrodes);
    
    for (unsigned int i = 0; i < dst->numElectrodes; ++i) 
    {
	for (unsigned int j = 0; j < src->numElectrodes; ++j)
	{
	    btVector3 diff = dst->electrodePositionsTransformed[i] - src->electrodePositionsTransformed[j];
	    float dist = diff.length();
	    contributions(i) += ISrc(j) / dist;
        }
    }
    
    std::cout << " adding contributions" << contributions(0) << " " << contributions(1) << " " << contributions(2) << " " << contributions(3) << " " << contributions(4) << std::endl;	    
    
    contributions /= 4.0 * M_PI * gamma;        
    IDst += dst->C0 * contributions;
}
