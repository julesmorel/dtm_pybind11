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

#ifndef LOCALREF_H
#define LOCALREF_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "../constant.h"

class localCell
{
public:

    localCell(){}
    localCell(const localCell &old_cell);
    localCell(const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, int plane);
    localCell(const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius=0., bool useWeight=false);
    localCell(pcl::PointXYZ center, const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius=0., bool useWeight=false);
    localCell(pcl::PointXYZ center, pcl::PointXYZ normal, const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius=0., bool useWeight=false);
    localCell(pcl::PointXYZ center, pcl::PointXYZ normal);

    //getter on attributes and results
    const Eigen::Vector3f& getCentroid() const {return this->centroid;}
    const Eigen::Vector3f& getU() const {return this->u;}
    const Eigen::Vector3f& getV() const {return this->v;}
    const Eigen::Vector3f& getW() const {return this->w;}
    const Eigen::Affine3f& getTransfo() const {return this->transfo;}
    const std::vector<double>& getCoeff() const {return this->coeff;}
    double getError()const {return this->error;}

    const Eigen::Matrix4f& getM1() const {return m1;}
    const Eigen::Vector4f& getM2() const {return m2;}

    const pcl::PointCloud<pcl::PointXYZI>& getLocalPts() const{return this->ptsLocalRef;}
    const pcl::PointCloud<pcl::PointXYZI>& getGlobalPts() const{return this->ptsGlobalRef;}

    //get the value of the implicit function at the point (x,y,z)
    float getValueOfImplicitFunction(float x, float y, float z) const;


protected:

    //indices of points inside this cell and a pointer to all the points
    pcl::PointCloud<pcl::PointXYZI>   ptsGlobalRef;
    pcl::PointCloud<pcl::PointXYZI>   ptsLocalRef;

    //local origin (centroid of input points)
    Eigen::Vector3f centroid;

    //local basis
    Eigen::Vector3f u;
    Eigen::Vector3f v;
    Eigen::Vector3f w;

    //Transformation from local basis to global basis
    Eigen::Affine3f transfo;

    //coefficient of the quadric
    std::vector<double> coeff;

    //in the global basis,
    //the equation of quatric is : trans(X)*m1*X + m2*X = 0
    Eigen::Matrix4f m1;
    Eigen::Vector4f m2;

    //error from the least square regression
    double error;

    //compute the centroid
    void computeCentroid();

    //compute the plane that approximates the data in the least square sense
    void getNormal();

    //average of pts to get w, then choose u and v to have a direct basis
    void getBasis();

    //transform the point cloud in the local basis
    void applyTransform();

    //get coef of quadric and error by using least square reg
    void computeLeastSquareReg(float radius, bool useWeight);
    void computeLeastSquareRegPlane();

    //compute coeffs of quadric in the global basis
    void getGlobalQuadric();
};

#endif // LOCALREF_H
