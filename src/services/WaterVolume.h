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


#ifndef WATER_VOLUME_H
#define WATER_VOLUME_H

#include "Service.h"

#include <functional>
#include "LinearMath/btVector3.h"

class WaterVolumeInterface;

typedef std::function< float (btVector3, float) > WaterVolumeHeightCallback;
typedef std::function< btVector3 (btVector3, float) > WaterVolumeCurrentCallback;
typedef std::function< float (btVector3, float) > WaterVolumeTemperatureCallback;
typedef std::function< float (btVector3, float) > WaterVolumePHCallback;
typedef std::function< float (btVector3, float) > WaterVolumeO2Callback;
typedef std::function< float (btVector3, float) > WaterVolumeTurbidityCallback;

class WaterVolume : public Service
{
public :

    // callbacks that should be implemented by user
    WaterVolumeHeightCallback getHeight;
    WaterVolumeCurrentCallback getCurrent;
    WaterVolumeTemperatureCallback getTemperature;
    WaterVolumePHCallback getPH;
    WaterVolumeO2Callback getO2;
    WaterVolumeO2Callback getTurbidity;

    float density = 1000.0;
    
    // Methods ==================

    WaterVolume();
    ~WaterVolume();

    void step ();
    void reset ();

    void setHeightCallback(WaterVolumeHeightCallback c);
    void setCurrentCallback(WaterVolumeCurrentCallback c);
    void setTemperatureCallback(WaterVolumeTemperatureCallback c);
    void setPHCallback(WaterVolumePHCallback c);
    void setO2Callback(WaterVolumeO2Callback c);
    void setTurbidityCallback(WaterVolumeTurbidityCallback c);
    void setDensity(float d);
    
};


#endif



