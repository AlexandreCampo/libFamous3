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

#ifndef DEVICE_ELECTRIC_SENSE_H
#define DEVICE_ELECTRIC_SENSE_H


#include "Device.h"

#include "PhysicsBullet.h"
#include "RenderOpenGLInterface.h"
#include "RenderOSGInterface.h"

//#include <list>
#include <vector>
#include <Eigen/Eigen>

class DeviceElectricSense : public Device, public btBroadphaseAabbCallback, public RenderOpenGLInterface, public RenderOSGInterface
{
public :
    PhysicsBullet* physics;
    btRigidBody* parentBody;
    btTransform localTransform;
    int collisionFilter;
    int collisionType;

    btRigidBody* collisionBody; 
    btCollisionShape* collisionShape;

    float maxRange;
    float minElectrodeDistance = 0.01;

    DeviceElectricSense(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType = 2^30, float maxRange = 1.0);
    ~DeviceElectricSense();

    void send (int content);
    bool receive (int& c);

    void actionStep ();
    void perceptionStep ();
    void reset();

//    virtual bool process (const btBroadphaseProxy *proxy);

    void draw (RenderOpenGL* r);
    void draw (RenderOSG* r);


    // methods & members for e-sense calculations
    // ------------------------------------------
    void addElectrode(btVector3 position);

    void setMinElectrodeDistance(float mindist);
    void setPolarization (Eigen::VectorXf& polarization);
    void setC0 (Eigen::MatrixXf& C0);
    virtual bool process (const btBroadphaseProxy *proxy);
    Eigen::VectorXf getCurrents();
    void addContribution(DeviceElectricSense* src, DeviceElectricSense* dst, Eigen::VectorXf& ISrc, Eigen::VectorXf& IDst);
    
    // this is water conductivity... hardcoded for now...
    float gamma=0.06;
    
    Eigen::MatrixXf C0;
    Eigen::VectorXf I;
    
    bool polarized;

    int numElectrodes;
    std::vector<btVector3> electrodePositions;
    std::vector<btVector3> electrodePositionsTransformed;
    bool transformed;
    
    std::vector<DeviceElectricSense*> activeDevices;
    std::vector<DeviceElectricSense*> passiveDevices;

    Eigen::VectorXf polarization;
    Eigen::VectorXf I0;

    Eigen::VectorXf inducedI;
    bool inducedICalculated;
    
};


#endif
