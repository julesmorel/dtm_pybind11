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

#ifndef QUADTREE_H
#define QUADTREE_H

#include <deque>

#include "quadleaf.h"
#include "constant.h"
#include "rectangle.h"

#include <pcl/kdtree/kdtree_flann.h>

class quadTree
{
public:

    quadTree(){}
    quadTree(const pcl::PointCloud<pcl::PointXYZI>  &pPts, float , int , float, rectangle rect);
    ~quadTree();

    const std::deque<quadLeaf*>& getListLeaf() const {return listLeaf;}

    struct findByCodeWithChildren {
        findByCodeWithChildren(const std::bitset<QUAD_MAX_LEVEL> & bitset, int lvl) : bitset(bitset),level(lvl) {}
        bool operator()(const quadLeaf *leaf) {
            return leaf->getLocationCode() == bitset && leaf->notToDivide && leaf->getLevel() == level;
        }
    private:
        std::bitset<QUAD_MAX_LEVEL> bitset;
        int level;
    };

    quadLeaf* findFirstLeafAboveThreshold();
    quadLeaf* findByBitset(std::bitset<QUAD_MAX_LEVEL> pCode, int pLevel);
    int getPositionInList(quadLeaf leaf);

    rectangle getRectangle(){return rectangleRoot;}

    void update();
    void buildLeafsSurface();

    const pcl::PointCloud<pcl::PointXYZI>& getPts() const {return pts;}
    const pcl::PointXYZ& getMin() const {return min;}
    const pcl::PointXYZ& getMax() const{return max;}

    std::bitset<QUAD_MAX_LEVEL> quadLocationAdd( std::bitset<QUAD_MAX_LEVEL> a, std::bitset<QUAD_MAX_LEVEL> b );

private:

    rectangle rectangleRoot;

    //-------------------------------------
    float errorThreshold;
    int nbrPointsMin;
    float minSizeLeaf;

    pcl::PointCloud<pcl::PointXYZI> pts;
    pcl::PointXYZ min,max;

    std::deque<quadLeaf*> listLeaf;

    void build();
    void approximateEmptyLeaves();
    void split(quadLeaf* pLeaf, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree);

    void incEqualSizeNeighbors(quadLeaf* pLeaf, bool checkIfBrother);
    bool isBrother(std::bitset<QUAD_MAX_LEVEL> pLeafA, std::bitset<QUAD_MAX_LEVEL> pLeafB);

    std::bitset<QUAD_MAX_LEVEL> add(std::bitset<QUAD_MAX_LEVEL> a, std::bitset<QUAD_MAX_LEVEL> b);
    std::bitset<QUAD_MAX_LEVEL> calcNeighborsEqualSize(std::bitset<QUAD_MAX_LEVEL> pLocation,std::bitset<QUAD_MAX_LEVEL> direction );
    std::bitset<QUAD_MAX_LEVEL> calcPosition(std::bitset<QUAD_MAX_LEVEL> pLocation, int pPosition );
    std::bitset<QUAD_MAX_LEVEL> getTx(int pLevelMax);
    std::bitset<QUAD_MAX_LEVEL> getTy(int pLevelMax);

    bool isLeafAboveThreshold();

    void printList();
};

#endif // QUADTREE_H
