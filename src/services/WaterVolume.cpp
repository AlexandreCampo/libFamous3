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


#include "Simulator.h"

#include "WaterVolume.h"
#include "WaterVolumeInterface.h"

// default callbacks
float waterVolumeHeightDefaultCallback(btVector3 pos, float time)
{
    return 1.0;
}

btVector3 waterVolumeCurrentDefaultCallback(btVector3 pos, float time)
{
    return btVector3(0,0,0);
}

float waterVolumeTemperatureDefaultCallback(btVector3 pos, float time)
{
    return 0;
}

float waterVolumePHDefaultCallback(btVector3 pos, float time)
{
    return 7;
}

float waterVolumeO2DefaultCallback(btVector3 pos, float time)
{
    return 0;
}

float waterVolumeTurbidityDefaultCallback(btVector3 pos, float time)
{
    return 0;
}

// methods

WaterVolume::WaterVolume ()
{
    getHeight = waterVolumeHeightDefaultCallback;
    getCurrent = waterVolumeCurrentDefaultCallback;
    getTemperature = waterVolumeTemperatureDefaultCallback;
    getPH = waterVolumePHDefaultCallback;
    getO2 = waterVolumeO2DefaultCallback;
    getTurbidity = waterVolumeTurbidityDefaultCallback;
}

void WaterVolume::setDensity (float density)
{
    this->density = density;
}

void WaterVolume::setHeightCallback(WaterVolumeHeightCallback c)
{
    this->getHeight = c;
}

void WaterVolume::setCurrentCallback(WaterVolumeCurrentCallback c)
{
    this->getCurrent = c;
}

void WaterVolume::setTemperatureCallback(WaterVolumeTemperatureCallback c)
{
    this->getTemperature = c;
}

void WaterVolume::setPHCallback(WaterVolumePHCallback c)
{
    this->getPH = c;
}

void WaterVolume::setO2Callback(WaterVolumeO2Callback c)
{
    this->getO2 = c;
}

void WaterVolume::setTurbidityCallback(WaterVolumeTurbidityCallback c)
{
    this->getTurbidity = c;
}

WaterVolume::~WaterVolume ()
{
}

void WaterVolume::step ()
{
}

void WaterVolume::reset ()
{
}


