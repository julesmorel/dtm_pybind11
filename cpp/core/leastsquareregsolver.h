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

#ifndef LEASTSQUAREREGSOLVER_H
#define LEASTSQUAREREGSOLVER_H

#include "constant.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class leastSquareRegSolver
{
public:
    static void solveQuadric(pcl::PointXYZ centroid, float radius, bool useWeight, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error);
    static void solvePlane(pcl::PointXYZ centroid, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error);
    static void solvePlaneLocal(pcl::PointXYZ centroid, pcl::PointCloud<pcl::PointXYZI>  pts, std::vector<double>* coeff, double* error);
};

#endif // LEASTSQUAREREGSOLVER_H
