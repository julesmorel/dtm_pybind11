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

#ifndef RECTANGLE_H
#define RECTANGLE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


// p3 -- p2
// |     |
// p0 -- p1

class rectangle
{
public:

    rectangle(){}
    rectangle(pcl::PointXYZ pp0, pcl::PointXYZ pp1, pcl::PointXYZ pp2, pcl::PointXYZ pp3);
    rectangle(pcl::PointCloud<pcl::PointXYZ>   pPtsRect);

    pcl::PointXYZ getP0() const {return p0;}
    pcl::PointXYZ getP1() const {return p1;}
    pcl::PointXYZ getP2() const {return p2;}
    pcl::PointXYZ getP3() const {return p3;}

    pcl::PointCloud<pcl::PointXYZ>& getRect(){return rect;}

    pcl::PointCloud<pcl::PointXYZ> getCoordSubRectangle0();
    pcl::PointCloud<pcl::PointXYZ> getCoordSubRectangle1();
    pcl::PointCloud<pcl::PointXYZ> getCoordSubRectangle2();
    pcl::PointCloud<pcl::PointXYZ> getCoordSubRectangle3();

    float getSizeMinSide();
    float getSizeMaxSide();
    float getSizeXSide();
    float getSizeYSide();

    void keepPointIn(pcl::PointCloud<pcl::PointXYZI>   pPts,pcl::PointCloud<pcl::PointXYZI>   &pPtsOut);

    pcl::PointXYZ getPointWithRatio(float dx, float dy);
    void getRatioWithPoint(pcl::PointXYZ p, float& dx, float& dy);
    pcl::PointXYZ getCenter(pcl::PointCloud<pcl::PointXYZI> pPts);

    void print();

private:

pcl::PointCloud<pcl::PointXYZ>   rect;
pcl::PointXYZ p0,p1,p2,p3;
pcl::PointXYZ p01,p12,p23,p30,pc;

void computeMiddlePoints();

};

#endif // RECTANGLE_H
