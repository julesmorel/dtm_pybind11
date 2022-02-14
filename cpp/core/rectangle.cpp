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

#include "rectangle.h"

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <iostream>

rectangle::rectangle(pcl::PointXYZ pp0, pcl::PointXYZ pp1, pcl::PointXYZ pp2, pcl::PointXYZ pp3) : p0(pp0), p1(pp1), p2(pp2), p3(pp3)
{
  rect.push_back(p0);
  rect.push_back(p1);
  rect.push_back(p2);
  rect.push_back(p3);
  computeMiddlePoints();
}

rectangle::rectangle(pcl::PointCloud<pcl::PointXYZ> pPts): rect(pPts)
{
  p0 = pPts.at(0);
  p1 = pPts.at(1);
  p2 = pPts.at(2);
  p3 = pPts.at(3);

  computeMiddlePoints();
}

void rectangle::computeMiddlePoints()
{
  p01.x=0.5*(p0.x+p1.x);
  p01.y=0.5*(p0.y+p1.y);
  p01.z=0.5*(p0.z+p1.z);

  p12.x=0.5*(p1.x+p2.x);
  p12.y=0.5*(p1.y+p2.y);
  p12.z=0.5*(p1.z+p2.z);

  p23.x=0.5*(p2.x+p3.x);
  p23.y=0.5*(p2.y+p3.y);
  p23.z=0.5*(p2.z+p3.z);

  p30.x=0.5*(p3.x+p0.x);
  p30.y=0.5*(p3.y+p0.y);
  p30.z=0.5*(p3.z+p0.z);

  pc.x=0.5*(p2.x+p0.x);
  pc.y=0.5*(p2.y+p0.y);
  pc.z=0.5*(p2.z+p0.z);
}

void rectangle::keepPointIn(pcl::PointCloud<pcl::PointXYZI> pPts,pcl::PointCloud<pcl::PointXYZI>   &pPtsOut)
{

   pcl::PointCloud<pcl::PointXYZI> hull_cloud;
   pcl::PointXYZI p0i,p1i,p2i,p3i;
   p0i.x=p0.x;
   p0i.y=p0.y;
   p0i.z=p0.z;
   p1i.intensity=0.;
   p1i.x=p1.x;
   p1i.y=p1.y;
   p1i.z=p1.z;
   p1i.intensity=0.;
   p2i.x=p2.x;
   p2i.y=p2.y;
   p2i.z=p2.z;
   p2i.intensity=0.;
   p3i.x=p3.x;
   p3i.y=p3.y;
   p3i.z=p3.z;
   p3i.intensity=0.;

   hull_cloud.push_back(p0i);
   hull_cloud.push_back(p1i);
   hull_cloud.push_back(p2i);
   hull_cloud.push_back(p3i);

   std::vector<pcl::Vertices> rect;
   pcl::Vertices vt;
   vt.vertices.push_back(0);
   vt.vertices.push_back(1);
   vt.vertices.push_back(2);
   vt.vertices.push_back(3);
   rect.push_back(vt);

   pcl::CropHull<pcl::PointXYZI> crop_filter;
   crop_filter.setInputCloud (pPts.makeShared());
   crop_filter.setHullCloud (hull_cloud.makeShared());
   crop_filter.setHullIndices (rect);
   crop_filter.setDim (2);
   crop_filter.filter (pPtsOut);
}

float rectangle::getSizeMinSide()
{
    float d1 = pcl::geometry::distance(p0,p1);
    float d2 = pcl::geometry::distance(p0,p3);
    float minDist=d1;
    if(d2<d1)minDist=d2;
    return minDist;
}

float rectangle::getSizeMaxSide()
{
  float d1 = pcl::geometry::distance(p0,p1);
  float d2 = pcl::geometry::distance(p0,p3);
  float maxDist=d1;
  if(d2>d1)maxDist=d2;
  return maxDist;
}

float rectangle::getSizeXSide()
{
    return pcl::geometry::distance(p0,p1);
}

float rectangle::getSizeYSide()
{
    return pcl::geometry::distance(p0,p3);
}

pcl::PointCloud<pcl::PointXYZ> rectangle::getCoordSubRectangle0()
{
  pcl::PointCloud<pcl::PointXYZ> sub;
  pcl::PointXYZ _p0(p0);
  pcl::PointXYZ _p1(p01);
  pcl::PointXYZ _p2(pc);
  pcl::PointXYZ _p3(p30);
  sub.push_back(_p0);
  sub.push_back(_p1);
  sub.push_back(_p2);
  sub.push_back(_p3);
  return sub;
}

pcl::PointCloud<pcl::PointXYZ> rectangle::getCoordSubRectangle1()
{
  pcl::PointCloud<pcl::PointXYZ> sub;
  pcl::PointXYZ _p0(p01);
  pcl::PointXYZ _p1(p1);
  pcl::PointXYZ _p2(p12);
  pcl::PointXYZ _p3(pc);
  sub.push_back(_p0);
  sub.push_back(_p1);
  sub.push_back(_p2);
  sub.push_back(_p3);
  return sub;
}

pcl::PointCloud<pcl::PointXYZ> rectangle::getCoordSubRectangle2()
{
  pcl::PointCloud<pcl::PointXYZ> sub;
  pcl::PointXYZ _p0(pc);
  pcl::PointXYZ _p1(p12);
  pcl::PointXYZ _p2(p2);
  pcl::PointXYZ _p3(p23);
  sub.push_back(_p0);
  sub.push_back(_p1);
  sub.push_back(_p2);
  sub.push_back(_p3);
  return sub;
}

pcl::PointCloud<pcl::PointXYZ> rectangle::getCoordSubRectangle3()
{
  pcl::PointCloud<pcl::PointXYZ> sub;
  pcl::PointXYZ _p0(p30);
  pcl::PointXYZ _p1(pc);
  pcl::PointXYZ _p2(p23);
  pcl::PointXYZ _p3(p3);
  sub.push_back(_p0);
  sub.push_back(_p1);
  sub.push_back(_p2);
  sub.push_back(_p3);
  return sub;
}

void rectangle::print()
{
  std::cout<<"p0 "<<p0.x<<" "<<p0.y<<" "<<p0.z<<std::endl;
  std::cout<<"p1 "<<p1.x<<" "<<p1.y<<" "<<p1.z<<std::endl;
  std::cout<<"p3 "<<p3.x<<" "<<p3.y<<" "<<p3.z<<std::endl;
}

pcl::PointXYZ rectangle::getPointWithRatio(float dx, float dy)
{
  Eigen::Vector3f u,v;
  u[0]=p1.x-p0.x;
  u[1]=p1.y-p0.y;
  u[2]=p1.z-p0.z;
  v[0]=p3.x-p0.x;
  v[1]=p3.y-p0.y;
  v[2]=p3.z-p0.z;

  Eigen::Vector3f comb=dx*u+dy*v;

  pcl::PointXYZ p;
  p.x=comb[0]+p0.x;
  p.y=comb[1]+p0.y;
  p.z=comb[2]+p0.z;

  return p;
}

void rectangle::getRatioWithPoint(pcl::PointXYZ p, float& dx, float& dy)
{
  pcl::PointXYZ u,v;
  u.x=p1.x-p0.x;
  u.y=p1.y-p0.y;
  u.z=p1.z-p0.z;
  float normU = sqrt(u.x*u.x+u.y*u.y+u.z*u.z);
  v.x=p3.x-p0.x;
  v.y=p3.y-p0.y;
  v.z=p3.z-p0.z;
  float normV = sqrt(v.x*v.x+v.y*v.y+v.z*v.z);

  dx = (u.x*(p.x-p0.x)+u.y*(p.y-p0.y)+u.z*(p.z-p0.z))/normU;
  dy = (v.x*(p.x-p0.x)+v.y*(p.y-p0.y)+v.z*(p.z-p0.z))/normV;
}

pcl::PointXYZ rectangle::getCenter(pcl::PointCloud<pcl::PointXYZI> pPts)
{
  pcl::PointXYZI BBMin, BBMax;
  pcl::getMinMax3D(pPts, BBMin, BBMax);
  pcl::PointXYZ center = getPointWithRatio(0.5,0.5);
  center.z = 0.5*(BBMin.z+BBMax.z);
  return center;
}
