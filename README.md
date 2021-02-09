# **Haptic Guides**

## Table of Contents

1. [Introduction](#1. Introduction)
2. [Requirements](#2. Requirements)
3. [License](#3. License)

## 1. Introduction

In the field of telemanipulation devices, where an operator controls a remote 
robotic arm, one of the most important focuses is on security. Up to date, most 
users depend on their own ability and on the assistance offered by these devices. 
The most common aids consist in including a camera, to provide greater visibility, 
and in systems that increase the sensitivity of the user movements. A possible 
help is to help the interaction of the user with small or difficult to be accessed 
areas, providing a support which concentrates the approach paths towards the 
considered zone. In addition, it allows to restrict the access to other potentially 
sensitive areas, that is, avoiding that the user touches them unintentionally. 
This is the goal of what we call haptic guides. In order to interact with them, 
a haptic device is required. The haptic device can constitute  the user interface 
within a telemanipulation system, while at the same time providing the possibility 
of touching these guides.
In order to develop such guides, we use techniques of virtual objects construction 
with funnel shape and collision detection algorithms between two virtual objects. 
A Kinect sensor is used for the selecting the delimited area, as desired by the 
user. Computer vision techniques like filtering, segmentation and edges detection 
of a point cloud are used for performing this area selection.

## 2. Requirements

### Hardware

* Kinect v1
* Delta Haptic Device 6

### Software

* CMake:

sudo apt-get install cmake

* libfreenect

## 3. License

Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author azubiri (azubiri@iri.upc.edu)
All rights reserved.

This file is part of Haptic guides project library
Haptic guides project library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>


