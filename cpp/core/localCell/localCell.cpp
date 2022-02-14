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

#include "localCell.h"

#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <gsl/gsl_multifit.h>

#include "../leastsquareregsolver.h"

localCell::localCell(const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius, bool useWeight) : ptsGlobalRef(pPtsGlobalRef)
{
    //std::cout<<"start localcell"<<std::endl;

    computeCentroid();
    getNormal();
    getBasis();
    applyTransform();

    computeLeastSquareReg(radius,useWeight);
    //std::cerr << "befor global quadric" << '\n';
    getGlobalQuadric();

    //std::cout<<"localcell ok!"<<std::endl;
}

localCell::localCell(const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, int plane) : ptsGlobalRef(pPtsGlobalRef)
{
    computeCentroid();
    getNormal();
    getBasis();
    applyTransform();
    computeLeastSquareRegPlane();
    getGlobalQuadric();
}

localCell::localCell(const localCell &old_cell)
{
    centroid = old_cell.getCentroid();
    u = old_cell.getU();
    v = old_cell.getV();
    w = old_cell.getW();
    m1 = old_cell.getM1();
    m2 = old_cell.getM2();
    coeff = old_cell.getCoeff();
    error = old_cell.getError();

    std::cout<<"localCell copy ok!"<<std::endl;
}

localCell::localCell(pcl::PointXYZ center,const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius, bool useWeight) : ptsGlobalRef(pPtsGlobalRef)
{
    centroid[0] = center.x;
    centroid[1] = center.y;
    centroid[2] = center.z;

    getNormal();
    getBasis();
    applyTransform();

    computeLeastSquareReg(radius,useWeight);
    getGlobalQuadric();
}

localCell::localCell(pcl::PointXYZ center, pcl::PointXYZ normal, const pcl::PointCloud<pcl::PointXYZI> &pPtsGlobalRef, float radius, bool useWeight) : ptsGlobalRef(pPtsGlobalRef)
{
    centroid[0] = center.x;
    centroid[1] = center.y;
    centroid[2] = center.z;

    w[0]=normal.x;
    w[1]=normal.y;
    w[2]=normal.z;
    w=w/w.norm();

    getBasis();
    applyTransform();

    computeLeastSquareReg(radius,useWeight);
    getGlobalQuadric();
}

localCell::localCell(pcl::PointXYZ center, pcl::PointXYZ normal)
{
    centroid[0] = center.x;
    centroid[1] = center.y;
    centroid[2] = center.z;

    w[0]=normal.x;
    w[1]=normal.y;
    w[2]=normal.z;
    w=w/w.norm();

    getBasis();
}

void localCell::getNormal()
{
    u.setZero(3);
    v.setZero(3);
    w.setZero(3);

    std::vector<double> coeffPlane;
    double errorPlane;
    pcl::PointXYZ cen;
    cen.x = centroid[0];
    cen.y = centroid[1];
    cen.z = centroid[2];
    leastSquareRegSolver::solvePlane(cen,ptsGlobalRef,&coeffPlane,&errorPlane);
    w[0]=-coeffPlane.at(0);
    w[1]=-coeffPlane.at(1);
    w[2]=1.;

    w=w/w.norm();
}

void localCell::getBasis()
{
    Eigen::Matrix3f R;
    R = Eigen::Quaternionf().setFromTwoVectors(Eigen::Vector3f::UnitZ(),w);

    u = R* Eigen::Vector3f::UnitX();
    v = R* Eigen::Vector3f::UnitY();

    pcl::getTransformationFromTwoUnitVectorsAndOrigin(v,w,centroid,transfo);
}

void localCell::computeCentroid()
{
    centroid.setZero(3);

    for(unsigned int i=0; i<ptsGlobalRef.size(); i++){
        centroid[0] += ptsGlobalRef.at(i).x/(float)ptsGlobalRef.size();
        centroid[1] += ptsGlobalRef.at(i).y/(float)ptsGlobalRef.size();
        centroid[2] += ptsGlobalRef.at(i).z/(float)ptsGlobalRef.size();
    }
}

void localCell::applyTransform()
{
    pcl::transformPointCloud(ptsGlobalRef,ptsLocalRef,transfo);
}

float localCell::getValueOfImplicitFunction(float x, float y, float z) const
{
    Eigen::Vector4f X(x,y,z,1);
    return (double)((X.transpose()*m1.matrix())*X) + (float)(m2.transpose() * X);
}

void localCell::computeLeastSquareReg(float radius, bool useWeight)
{
    pcl::PointXYZ cen;
    cen.x = centroid[0];
    cen.y = centroid[1];
    cen.z = centroid[2];
    leastSquareRegSolver::solveQuadric(cen, radius , useWeight, ptsLocalRef, &this->coeff,&this->error);
}

void localCell::computeLeastSquareRegPlane()
{
    pcl::PointXYZ cen;
    cen.x = centroid[0];
    cen.y = centroid[1];
    cen.z = centroid[2];
    leastSquareRegSolver::solvePlaneLocal(cen, ptsLocalRef, &this->coeff,&this->error);
}

void localCell::getGlobalQuadric()
{
    Eigen::Matrix4f m1Local;
    Eigen::Vector4f m2Local(coeff.at(3),coeff.at(4),-1,coeff.at(5));
    m1Local<<coeff.at(0),0.5*coeff.at(1),0.,0.,
             0.5*coeff.at(1),coeff.at(2),0.,0.,
             0.,0.,0.,0.,
             0.,0.,0.,0.;
    //std::cout << "transfo: " << std::endl << transfo.matrix() << std::endl;
    Eigen::Matrix4f transT = transfo.matrix().transpose();
    //std::cout << "transfo T: " << std::endl << transT << std::endl;
    Eigen::Matrix4f transTxm1Local = transT * m1Local;
    //std::cout << "transTxm1Local: " << std::endl << transTxm1Local << std::endl;
    Eigen::Matrix4f tmp = transTxm1Local * transfo.matrix();
    //std::cout << "tmp: " << std::endl << tmp << std::endl;
    //m1 = tmp;
    m1<<tmp(0),tmp(1),tmp(2),tmp(3),tmp(4),tmp(5),tmp(6),tmp(7),tmp(8),tmp(9),tmp(10),tmp(11),tmp(12),tmp(13),tmp(14),tmp(15);
    //std::cout<<"4"<<std::endl;
    m2 = m2Local.transpose() * transfo.matrix();
}
