/****************************************************************************
 Copyright (C) 2017 Jules Morel

 Contact : jules.morel@ifpindia.org

 Developers : Jules MOREL (IFP LSIS)

 This file is part of PluginIFPLSIS library.

 PluginIFPLSIS is free library: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PluginIFPLSIS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.
*****************************************************************************/

#ifndef CONSTANT_H
#define CONSTANT_H

#endif // CONSTANT_H

//QTLCLD position
#define SOUTH_WEST 0
#define SOUTH_EAST 1
#define NORTH_WEST 2
#define NORTH_EAST 3

//Precalculated QTLCLD direction increments for r = 16 = max level
#define EAST_NEIGHBOR 0x01
#define NORTH_NEIGHBOR 0x02
#define WEST_NEIGHBOR 0x55555555
#define SOUTH_NEIGHBOR 0xAAAAAAAA

//Maximum division level of the tree
#define QUAD_MAX_LEVEL 16

//Number of coefficient of the quadric
#define NBR_COEFS 6

#define ERROR_LIMIT_ON_LEAF 0.8
#define ERROR_LIMIT_BETWEEN_NEIGHBOR 10.0
