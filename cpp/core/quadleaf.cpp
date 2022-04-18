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

#include "quadleaf.h"
#include "wendlandRbf.h"

#include <exception>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <fstream>

int quadLeaf::levelMax = 1;

quadLeaf::quadLeaf(quadLeaf* pParent, rectangle pRect, int pLevel, std::bitset<QUAD_MAX_LEVEL> pLocationCode, pcl::PointCloud<pcl::PointXYZI> &pPts , int nbrPointsMin, float minSizeLeaf, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree) : quadrant(pRect)
{
  //check how many points are actually in the leaf
  pcl::PointCloud<pcl::PointXYZI> pointsInLeaf;
  quadrant.keepPointIn(pPts,pointsInLeaf);
  //std::cout<<"   "<<pointsInLeaf.size()<<std::endl;
  /*if(pPts.size()==45 && pointsInLeaf.size()==0)
  {
    std::ofstream outfile;
    outfile.open("test.xyz", std::ios_base::app);
    for(int i=0;i<pPts.size();i++)
    {
        pcl::PointXYZI p = pPts.at(i);
        outfile <<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    }

    std::ofstream outfile2;
    outfile2.open("rect_test.xyz", std::ios_base::app);
    outfile2 <<quadrant.getP0().x<<" "<<quadrant.getP0().y<<" "<<quadrant.getP0().z<<std::endl;
    outfile2 <<quadrant.getP1().x<<" "<<quadrant.getP1().y<<" "<<quadrant.getP1().z<<std::endl;
    outfile2 <<quadrant.getP2().x<<" "<<quadrant.getP2().y<<" "<<quadrant.getP2().z<<std::endl;
    outfile2 <<quadrant.getP3().x<<" "<<quadrant.getP3().y<<" "<<quadrant.getP3().z<<std::endl;

  }*/

  //if the leaf is empty, let's just don't divide it more
  /*if(pointsInLeaf.size()==0){
    empty=true;
    notToDivide = true;
  //if it is not empty we compute a local approximation
}else{*/
    empty=false;
    if(pointsInLeaf.size()<nbrPointsMin || quadrant.getSizeMinSide()<minSizeLeaf)
    {
      throw std::domain_error ("Not enough points");
    }else{

      //we consider the point in a sphere centered on that leaf
      //and compute the local approximation
      keepPointsInSphere(kdtree,pPts,pointsInLeaf);
      updateBoundingBox(ptsSub);
    }
    notToDivide = false;
  //}

  //compute location code and relation to neighbors
  level = pLevel;
  parent = pParent;
  locationCode = pLocationCode;
  getPosition();
  calcDeltaNeighbors();
  checkIsOnBorder();
  isRoot = false;
}

void quadLeaf::approximate()
{
  cell = new localCell (ptsSub,0.,true);

  //Test extrem values of local patch coeffs
  std::vector<double> coef = cell->getCoeff();
  if(fabs(coef.at(0)*coef.at(1)*coef.at(2))>0.1)
  {
    flag=true;
  }else{
    flag=false;
  }

  ptsSub.clear();
}

quadLeaf::~quadLeaf()
{
    //delete localCell;
}

void quadLeaf::calcDeltaNeighbors()
{
    if(parent!=NULL)
    {
        deltaLevelEast = 0;
        deltaLevelNorth = 0;
        deltaLevelWest = 0;
        deltaLevelSouth = 0;

        if(position==SOUTH_WEST)
        {
            if(parent->deltaLevelWest == INT_MAX)
            {
                deltaLevelWest=INT_MAX;
            }else{
                deltaLevelWest=parent->deltaLevelWest-1;
            }
            if(parent->deltaLevelSouth == INT_MAX)
            {
                deltaLevelSouth=INT_MAX;
            }else{
                deltaLevelSouth=parent->deltaLevelSouth-1;
            }
        }
        if(position==SOUTH_EAST)
        {
            if(parent->deltaLevelSouth == INT_MAX)
            {
                deltaLevelSouth=INT_MAX;
            }else{
                deltaLevelSouth=parent->deltaLevelSouth-1;
            }
            if(parent->deltaLevelEast == INT_MAX)
            {
                deltaLevelEast=INT_MAX;
            }else{
                deltaLevelEast=parent->deltaLevelEast-1;
            }

        }
        if(position==NORTH_WEST)
        {
            if(parent->deltaLevelNorth == INT_MAX)
            {
                deltaLevelNorth=INT_MAX;
            }else{
                deltaLevelNorth=parent->deltaLevelNorth-1;
            }
            if(parent->deltaLevelWest == INT_MAX)
            {
                deltaLevelWest=INT_MAX;
            }else{
                deltaLevelWest=parent->deltaLevelWest-1;
            }
        }
        if(position==NORTH_EAST)
        {
            if(parent->deltaLevelEast == INT_MAX)
            {
                deltaLevelEast=INT_MAX;
            }else{
                deltaLevelEast=parent->deltaLevelEast-1;
            }
            if(parent->deltaLevelNorth == INT_MAX)
            {
                deltaLevelNorth=INT_MAX;
            }else{
                deltaLevelNorth=parent->deltaLevelNorth-1;
            }
        }
    }else{
        deltaLevelEast = INT_MAX;
        deltaLevelNorth = INT_MAX;
        deltaLevelWest = INT_MAX;
        deltaLevelSouth = INT_MAX;
    }

}

void quadLeaf::getPosition()
{
    if(!locationCode.test(1) & !locationCode.test(0))
    {
        position=SOUTH_WEST;
    }
    else if(!locationCode.test(1) & locationCode.test(0))
    {
        position=SOUTH_EAST;
    }
    else if(locationCode.test(1) & !locationCode.test(0))
    {
        position=NORTH_WEST;
    }
    else if(locationCode.test(1) & locationCode.test(0))
    {
        position=NORTH_EAST;
    }
}

void quadLeaf::updateBoundingBox(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    pcl::PointXYZI BBMini, BBMaxi;
    pcl::getMinMax3D (cloud, BBMini, BBMaxi);
    BBMin.x = BBMini.x;
    BBMin.y = BBMini.y;
    BBMin.z = BBMini.z;
    BBMax.x = BBMaxi.x;
    BBMax.y = BBMaxi.y;
    BBMax.z = BBMaxi.z;
}

void quadLeaf::fillScalarField(scalarField &field)
{
    pcl::PointXYZ min = field.getBBmin();

    int nbDivX = field.getXNode();
    int nbDivY = field.getYNode();
    int nbDivZ = field.getZNode();

    for(int k=0;k<nbDivZ+1;k++)
    {
        for(int j=0;j<nbDivY+1;j++)
        {
            for(int i=0;i<nbDivX+1;i++)
            {
                pcl::PointXYZ p = field.getPointFromIndex(i,j,k);

                Eigen::Vector4f X(p.x,p.y,p.z,1);
                float res = (float)((X.transpose()*getLocalCell()->getM1().matrix())*X) + (float)(getLocalCell()->getM2().transpose() * X);
                field.setValue(i,j,k,res);
            }
        }
    }
}

bool quadLeaf::isOnBorder()
{
    if(deltaLevelEast==INT_MAX || deltaLevelNorth==INT_MAX || deltaLevelSouth==INT_MAX || deltaLevelWest==INT_MAX)
    {
        return true;
    }else{
        return false;
    }
}

float quadLeaf::distancePointToSurface(pcl::PointXYZ pt)
{
    Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
    return 0.;//(float)((X.transpose()*getLocalCell().getM1().matrix())*X) + (float)(getLocalCell().getM2().transpose() * X);
}

void quadLeaf::keepPointsInSphere(pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree, pcl::PointCloud<pcl::PointXYZI> &pCloud,pcl::PointCloud<pcl::PointXYZI> &pointsInLeaf)
{
    //here we fix the radius
    radius = quadrant.getSizeMaxSide()*sqrt(3)*0.75;//+1;

    //pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    //kdtree.setInputCloud (pCloud.makeShared());

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    center = quadrant.getCenter(pointsInLeaf);
    pcl::PointXYZI c;
    c.x=center.x;
    c.y=center.y;
    c.z=center.z;
    c.intensity=0.;

    if ( kdtree.radiusSearch (c, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
            pcl::PointXYZI pt;
            pt.x = pCloud.at(pointIdxRadiusSearch[i]).x;
            pt.y = pCloud.at(pointIdxRadiusSearch[i]).y;
            pt.z = pCloud.at(pointIdxRadiusSearch[i]).z;
            float r = sqrt(pointRadiusSquaredDistance.at(i))/radius;
            pt.intensity = pCloud.at(pointIdxRadiusSearch[i]).intensity*wendlandRbf::getValueRbfWendland(r);
            ptsSub.push_back(pt);
        }
    }
}

void quadLeaf::checkIsOnBorder()
{
  border=false;
  if(parent!=NULL){
    if(parent->isBorder()){
      if(parent->getPositionCode()==SOUTH_WEST||parent->getPositionCode()==SOUTH_EAST){
        if(position==SOUTH_WEST||position==SOUTH_EAST){
          border=true;
        }
      }
      if(parent->getPositionCode()==NORTH_WEST||parent->getPositionCode()==NORTH_EAST){
        if(position==NORTH_WEST||position==NORTH_EAST){
          border=true;
        }
      }
      if(parent->getPositionCode()==NORTH_WEST||parent->getPositionCode()==SOUTH_WEST){
        if(position==NORTH_WEST||position==SOUTH_WEST){
          border=true;
        }
      }
      if(parent->getPositionCode()==NORTH_EAST||parent->getPositionCode()==SOUTH_EAST){
        if(position==NORTH_EAST||position==SOUTH_EAST){
          border=true;
        }
      }
    }
  }else{
    border=true;
  }
}

void quadLeaf::fillWithNeighbors(pcl::PointCloud<pcl::PointXYZI>& pCloud)
{
  //std::cout<<" Rebuilding : "<<locationCode.to_string()<<std::endl;
  //here we fix the radius
  radius = quadrant.getSizeMaxSide()*sqrt(3)+2;//*0.75;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (pCloud.makeShared());

  center = quadrant.getCenter(pCloud);
  pcl::PointXYZI c;
  c.x=center.x;
  c.y=center.y;
  c.z=center.z;
  c.intensity=0.;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if ( kdtree.radiusSearch (c, 3*radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
          pcl::PointXYZI pt;
          pt.x = pCloud.at(pointIdxRadiusSearch[i]).x;
          pt.y = pCloud.at(pointIdxRadiusSearch[i]).y;
          pt.z = pCloud.at(pointIdxRadiusSearch[i]).z;
          float r = sqrt(pointRadiusSquaredDistance.at(i))/radius;
          pt.intensity = pCloud.at(pointIdxRadiusSearch[i]).intensity*wendlandRbf::getValueRbfWendland(r);
          ptsSub.push_back(pt);
      }
  }
  center = quadrant.getCenter(ptsSub);
  cell = new localCell (ptsSub,0);
  empty=false;
}

void quadLeaf::printDeltaLevel()
{
  std::cout<<deltaLevelEast<<" "<<deltaLevelNorth<<" "<<deltaLevelWest<<" "<<deltaLevelSouth<<" "<<isOnBorder()<<std::endl;
}
