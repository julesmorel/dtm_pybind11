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

#ifndef QUADLEAF_H
#define QUADLEAF_H

#include <vector>
#include <bitset>

#include "rectangle.h"
#include "localCell/localCell.h"
#include "constant.h"
#include "../plot/implicitfunction.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class quadLeaf : public implicitFunction
{
public:
    quadLeaf(quadLeaf* pParent, rectangle pRect, int pLevel, std::bitset<QUAD_MAX_LEVEL> pLocationCode, pcl::PointCloud<pcl::PointXYZI> &pPts , int nbrPointsMin, float minSizeLeaf, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree);
    ~quadLeaf();

    static int levelMax;

    bool isRoot;
    bool notToDivide;
    int deltaLevelEast,deltaLevelNorth,deltaLevelWest,deltaLevelSouth;

    void setRadius(float pRad){radius = pRad;}
    void setNeighbors(std::vector<quadLeaf*> pNeighbors){neighbors = pNeighbors;}

    bool filterWithHisto(float threshold, float histoRange);

    void updateBoundingBox(pcl::PointXYZ pMin, pcl::PointXYZ pMax){BBMin=pMin;BBMax=pMax;}
    void updateBoundingBox(pcl::PointCloud<pcl::PointXYZI> &pCloud);
    pcl::PointXYZ& getBoundingBoxMin(){return BBMin;}
    pcl::PointXYZ& getBoundingBoxMax(){return BBMax;}

    bool isOnBorder();
    float distancePointToSurface(pcl::PointXYZ pt);


    std::vector<quadLeaf*>& getNeighbors(){return neighbors;}

    localCell* getLocalCell() {return cell;}
    rectangle& getQuadrant() {return quadrant;}
    float getRadius()const {return radius;}
    const std::bitset<QUAD_MAX_LEVEL>& getLocationCode() const {return locationCode;}
    int getLevel() const {return level;}
    const quadLeaf* getParent(){return parent;}
    pcl::PointXYZ getCenter(){return center;}

    bool isEmpty(){return empty;}
    void fillWithNeighbors(pcl::PointCloud<pcl::PointXYZI>& pPts);

    virtual void fillScalarField(scalarField &field);

    void approximate();

    bool getFlag(){return flag;}
    void setFlag(){flag=true;}

private:

    std::bitset<QUAD_MAX_LEVEL> locationCode;
    int level;
    int position;

    bool empty;
    bool flag;

    rectangle quadrant;
    quadLeaf* parent;

    pcl::PointCloud<pcl::PointXYZI>   ptsSub;

    localCell * cell;

    mutable std::vector<quadLeaf*> neighbors;
    mutable float radius;
    pcl::PointXYZ center;

    void getPosition();
    void calcDeltaNeighbors();

    void keepPointsInSphere(pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree, pcl::PointCloud<pcl::PointXYZI> &pCloud,pcl::PointCloud<pcl::PointXYZI> &pleaf);

};

#endif // QUADLEAF_H
