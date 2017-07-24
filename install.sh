#!/usr/bin/env bash

# check for dependencies
sudo apt-get install make g++ libgsl0-dev libtinyxml-dev libboost-program-options-dev libbullet-dev premake4 chipmunk-dev libopenscenegraph-dev freeglut3-dev libeigen3-dev

# build
premake4 gmake
make
premake4 debpackage

#install
sudo dpkg -i libfamous-3.0.deb
