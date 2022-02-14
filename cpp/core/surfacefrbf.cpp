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

#include "surfacefrbf.h"
#include "wendlandRbf.h"

#include <pcl/common/common.h>

#include <time.h>

#include <fstream>

#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_multimin.h>

surfaceFrbf::surfaceFrbf(const pcl::PointCloud<pcl::PointXYZI> &pCloud, int nbrPointsMin, float minSizeLeaf, pcl::PointCloud<pcl::PointXYZ> prect, float pThreshold)
{
    //clock_t t = clock();
    rectangle tmp(prect);
    rect = tmp;
    tree = new quadTree(pCloud,pThreshold, nbrPointsMin, minSizeLeaf, rect);
    /*t = clock() - t;
    std::cout<<"tree : "<<((float)t)/CLOCKS_PER_SEC<<" s"<<std::endl;
    t = clock();*/


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pts_ptr (new pcl::PointCloud<pcl::PointXYZI>(pCloud));
    pcl::PointXYZI BBMini, BBMaxi;
    pcl::getMinMax3D (*cloud_pts_ptr, BBMini, BBMaxi);
    BBMin.x = BBMini.x;
    BBMin.y = BBMini.y;
    BBMin.z = BBMini.z;
    BBMax.x = BBMaxi.x;
    BBMax.y = BBMaxi.y;
    BBMax.z = BBMaxi.z;
    BBMin.z = tree->getMin().z-1.;
    BBMax.z = tree->getMax().z+1.;
}

  void surfaceFrbf::fillScalarField(scalarField &field)
  {
    pcl::PointXYZ min = field.getRectangle().getP0();

    int nbDivX = field.getXNode();
    int nbDivY = field.getYNode();
    int nbDivZ = field.getZNode();

    float sizeCellX = field.getSizeCellX();
    float sizeCellY = field.getSizeCellY();
    float sizeCellZ = field.getSizeCellZ();

    //std::cout<<"sizeCell "<<sizeCellX<<" "<<sizeCellY<<" "<<sizeCellZ<<std::endl;

    //std::ofstream fileStream("test5.xyz", std::ios::out | std::ios::trunc);

    for(std::size_t m=0;m<tree->getListLeaf().size();m++)
    {
      quadLeaf* leaf = tree->getListLeaf().at(m);
      if(!leaf->isEmpty()){

        int coordCenterX, coordCenterY, coordCenterZ;

        float dx,dy,dz;

        pcl::PointXYZ c = leaf->getCenter();

        field.getRectangle().getRatioWithPoint(c, dx, dy);
        dz = leaf->getCenter().z-field.getZmin();

        coordCenterX = (int)(dx/sizeCellX);
        coordCenterY = (int)(dy/sizeCellY);
        coordCenterZ = (int)(dz/sizeCellZ);

        int stepX = (int)(leaf->getRadius()/sizeCellX)+1;
        int stepY = (int)(leaf->getRadius()/sizeCellY)+1;
        int stepZ = (int)(leaf->getRadius()/sizeCellZ)+1;

        //fileStream<<c.x<<" "<<c.y<<" "<<c.z<<" "<<dx<<" "<<dy<<" "<<dz<<std::endl;

        for(int k=coordCenterZ-stepZ;k<coordCenterZ+stepZ;k++)
        {
          for(int j=coordCenterY-stepY;j<coordCenterY+stepY;j++)
          {
            for(int i=coordCenterX-stepX;i<coordCenterX+stepX;i++)
            {
              if(i>=0 && i<nbDivX && j>=0 && j<nbDivY && k>=0 && k<nbDivZ){

                pcl::PointXYZ p = field.getPointFromIndex(i,j,k);
                //fileStream<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;

                float sum = 0.;
                if(field.getValue(i,j,k) != scalarField::NODATA)
                {
                  sum = field.getValue(i,j,k);
                }

                bool hasBeenUpdated = false;

                float tmpx = (leaf->getCenter().x-p.x);
                float tmpy = (leaf->getCenter().y-p.y);
                float tmpz = (leaf->getCenter().z-p.z);
                float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);

                float r = distR/leaf->getRadius();

                if(r<1.)
                {
                  float res = leaf->getLocalCell()->getValueOfImplicitFunction(p.x,p.y,p.z);
                  //here we're supposed to divide by the sum of the csrbf at the point p
                  //we don't do it for speed sake, the 0-level set is the same anyway
                  sum += (res)*wendlandRbf::getValueRbfWendland(r);///getSumRbfAtPoint(p);

                  hasBeenUpdated = true;
                }

                if(hasBeenUpdated)field.setValue(i,j,k,sum);
              }
            }
          }
        }
      }
  }
}

double surfaceFrbf::computeImplicitFctValue(pcl::PointXYZ pt)
{
  double val=0.;
  for(std::size_t m=0;m<tree->getListLeaf().size();m++)
  {
    quadLeaf* leaf = tree->getListLeaf().at(m);
    if(!leaf->isEmpty()){
      float tmpx = (leaf->getCenter().x-pt.x);
      float tmpy = (leaf->getCenter().y-pt.y);
      float tmpz = (leaf->getCenter().z-pt.z);
      float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      float r = distR/leaf->getRadius();
      if(r<1.)
      {
        Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
        double coefrbf = wendlandRbf::getValueRbfWendland(r)/getSumRbfAtPoint(pt);
        double res = leaf->getLocalCell()->getValueOfImplicitFunction(pt.x,pt.y,pt.z);
        val += (res)*coefrbf;
      }
    }
  }
  return val;
}

bool surfaceFrbf::computeImplicitFctValueAndPseudoGrad(pcl::PointXYZ pt, double& val, pcl::Normal& grad)
{
  bool updated=false;
  for(std::size_t m=0;m<tree->getListLeaf().size();m++)
  {
    quadLeaf* leaf = tree->getListLeaf().at(m);
    if(!leaf->isEmpty()){
      float tmpx = (leaf->getCenter().x-pt.x);
      float tmpy = (leaf->getCenter().y-pt.y);
      float tmpz = (leaf->getCenter().z-pt.z);
      float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      float r = distR/leaf->getRadius();

      if(r<1.)
      {
        Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
        Eigen::Vector4f resv = ((2*leaf->getLocalCell()->getM1().matrix())*X)+(leaf->getLocalCell()->getM2());
        double coefrbf = wendlandRbf::getValueRbfWendland(r);///getSumRbfAtPoint(pt);

        double res = leaf->getLocalCell()->getValueOfImplicitFunction(pt.x,pt.y,pt.z);

        val += (res)*coefrbf;

        grad.normal_x+=(resv[0])*coefrbf;
        grad.normal_y+=(resv[1])*coefrbf;
        grad.normal_z+=(resv[2])*coefrbf;

        updated=true;
      }
    }
  }
  float normGrad = sqrt(grad.normal_x*grad.normal_x+grad.normal_y*grad.normal_y+grad.normal_z*grad.normal_z);
  grad.normal_x=grad.normal_x/normGrad;
  grad.normal_y=grad.normal_y/normGrad;
  grad.normal_z=grad.normal_z/normGrad;

  return updated;
}

pcl::Normal surfaceFrbf::computeImplicitFctGrad(pcl::PointXYZ pt)
{
  pcl::Normal grad;
  for(std::size_t m=0;m<tree->getListLeaf().size();m++)
  {
    quadLeaf* leaf = tree->getListLeaf().at(m);
    if(!leaf->isEmpty()){
      float tmpx = (leaf->getCenter().x-pt.x);
      float tmpy = (leaf->getCenter().y-pt.y);
      float tmpz = (leaf->getCenter().z-pt.z);
      float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
      float r = distR/leaf->getRadius();

      if(r<1.)
      {
        Eigen::Vector4f X(pt.x,pt.y,pt.z,1);
        Eigen::Vector4f resv = ((2*leaf->getLocalCell()->getM1().matrix())*X)+(leaf->getLocalCell()->getM2());
        double coefrbf = wendlandRbf::getValueRbfWendland(r);///getSumRbfAtPoint(pt);
        grad.normal_x+=(resv[0])*coefrbf;
        grad.normal_y+=(resv[1])*coefrbf;
        grad.normal_z+=(resv[2])*coefrbf;
      }
    }
  }
  float normGrad = sqrt(grad.normal_x*grad.normal_x+grad.normal_y*grad.normal_y+grad.normal_z*grad.normal_z);
  grad.normal_x=grad.normal_x/normGrad;
  grad.normal_y=grad.normal_y/normGrad;
  grad.normal_z=grad.normal_z/normGrad;

  return grad;
}

double surfaceFrbf::getSumRbfAtPoint(pcl::PointXYZ pt)
{
  double sum=0.;
  for(std::size_t m=0;m<tree->getListLeaf().size();m++)
  {
    quadLeaf* leaf = tree->getListLeaf().at(m);
    float tmpx = (leaf->getCenter().x-pt.x);
    float tmpy = (leaf->getCenter().y-pt.y);
    float tmpz = (leaf->getCenter().z-pt.z);
    float distR = sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz);
    float r = distR/leaf->getRadius();
    sum += wendlandRbf::getValueRbfWendland(r);
  }
  return sum;
}
