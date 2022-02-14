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

#ifndef SURFACE_H
#define SURFACE_H

#include "quadtree.h"

#include <deque>

#include "../plot/implicitfunction.h"

struct parametersNewtonFRBF {
    std::deque<quadLeaf*> listSph_r;
    pcl::PointXYZ po_r;
    pcl::Normal no_r;
};

class surfaceFrbf : public implicitFunction
{
public:

    surfaceFrbf(){}
    surfaceFrbf(const pcl::PointCloud<pcl::PointXYZI> &pCloud, int nbrPointsMin, float minSizeLeaf, pcl::PointCloud<pcl::PointXYZ> rect, float pThreshold=0.00001);
    quadTree* getTree() const {return tree;}

    virtual void fillScalarField(scalarField &field);

    double computeImplicitFctValue(pcl::PointXYZ pt);
    pcl::Normal computeImplicitFctGrad(pcl::PointXYZ pt);
    bool computeImplicitFctValueAndPseudoGrad(pcl::PointXYZ pt, double& val, pcl::Normal& grad);

private:

    quadTree * tree;
    rectangle rect;
    std::vector<float> valuesImplicitFunction;

    double getSumRbfAtPoint(pcl::PointXYZ p);
};

#endif // SURFACE_H
