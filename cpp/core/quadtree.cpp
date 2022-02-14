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

#include "quadtree.h"

#include <algorithm>
#include <iostream>
#include <exception>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

quadTree::quadTree(const pcl::PointCloud<pcl::PointXYZI>  &pPts, float pThreshold, int pNbrPointsMin, float pMinSizeLeaf, rectangle rect) : pts(pPts), errorThreshold(pThreshold),nbrPointsMin(pNbrPointsMin),minSizeLeaf(pMinSizeLeaf),rectangleRoot(rect)
{
    build();
    /*std::cout<<"Quadtree built, filling empty cells now ..."<<std::endl;
    approximateEmptyLeaves();
    std::cout<<"... Done!"<<std::endl;*/
}

quadTree::~quadTree()
{

}

void quadTree::build()
{
    //std::cout<<pts.size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pts_ptr (new pcl::PointCloud<pcl::PointXYZI>(pts));
    //pcl::getMinMax3D (*cloud_pts_ptr, min, max);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (pts.makeShared());

    pcl::PointXYZI BBMini, BBMaxi;
    pcl::getMinMax3D (*cloud_pts_ptr, BBMini, BBMaxi);
    min.x = BBMini.x;
    min.y = BBMini.y;
    min.z = BBMini.z;
    max.x = BBMaxi.x;
    max.y = BBMaxi.y;
    max.z = BBMaxi.z;
    //rectangle rectangleRoot = rectangle(Ptsrect);
    //rectangleRoot.print();
    std::bitset<QUAD_MAX_LEVEL> bs((long)0);
    quadLeaf* root = new quadLeaf(NULL,rectangleRoot,0,bs,pts,nbrPointsMin,minSizeLeaf,kdtree);
    root->isRoot = true;
    listLeaf.push_front(root);

    while(isLeafAboveThreshold())
    {
        quadLeaf* firstLeaf = findFirstLeafAboveThreshold();
        try{

            split(firstLeaf,kdtree);
            firstLeaf->notToDivide = true;
            if(!firstLeaf->isRoot)incEqualSizeNeighbors(firstLeaf,false);
            listLeaf.erase(listLeaf.begin()+getPositionInList(*firstLeaf));

            //printList();

        }catch(std::domain_error& e)
        {
            firstLeaf->notToDivide = true;
        }
    }

    std::deque<quadLeaf*>::iterator it = listLeaf.begin();
    while (it != listLeaf.end())
    {
        (*it)->approximate();
        it++;
    }
}

void quadTree::split(quadLeaf* pLeaf, pcl::KdTreeFLANN<pcl::PointXYZI> &kdtree)
{
    double errorParent = pLeaf->getLocalCell()->getError();

    rectangle rectangleSW(pLeaf->getQuadrant().getCoordSubRectangle0());
    quadLeaf* lSW = new quadLeaf(pLeaf,rectangleSW,pLeaf->getLevel()+1,calcPosition(pLeaf->getLocationCode(),SOUTH_WEST),pts,nbrPointsMin,minSizeLeaf,kdtree);

    rectangle rectangleSE(pLeaf->getQuadrant().getCoordSubRectangle1());
    quadLeaf* lSE = new quadLeaf(pLeaf,rectangleSE,pLeaf->getLevel()+1,calcPosition(pLeaf->getLocationCode(),SOUTH_EAST),pts,nbrPointsMin,minSizeLeaf,kdtree);

    rectangle rectangleNW(pLeaf->getQuadrant().getCoordSubRectangle2());
    quadLeaf* lNW = new quadLeaf(pLeaf,rectangleNW,pLeaf->getLevel()+1,calcPosition(pLeaf->getLocationCode(),NORTH_WEST),pts,nbrPointsMin,minSizeLeaf,kdtree);

    rectangle rectangleNE(pLeaf->getQuadrant().getCoordSubRectangle3());
    quadLeaf* lNE = new quadLeaf(pLeaf,rectangleNE,pLeaf->getLevel()+1,calcPosition(pLeaf->getLocationCode(),NORTH_EAST),pts,nbrPointsMin,minSizeLeaf,kdtree);

    double errorSW = lSW->getLocalCell()->getError();
    double errorSE = lSE->getLocalCell()->getError();
    double errorNW = lNW->getLocalCell()->getError();
    double errorNE = lNE->getLocalCell()->getError();

    /*if(errorParent<errorSW || errorParent<errorSE || errorParent<errorNW || errorParent<errorNE)
    {
        std::cout<<"Errors: "<<errorParent<<" | "<<errorSW<<" "<<errorSE<<" "<<errorNW<<" "<<errorNE<<std::endl;
        printList();
    }*/

    if(pLeaf->getLevel()+1 > quadLeaf::levelMax)
    {
        quadLeaf::levelMax=pLeaf->getLevel()+1;
    }

    listLeaf.push_back(lSW);
    listLeaf.push_back(lSE);
    listLeaf.push_back(lNW);
    listLeaf.push_back(lNE);

    if(!pLeaf->isRoot){
        incEqualSizeNeighbors(lSW,true);
        incEqualSizeNeighbors(lSE,true);
        incEqualSizeNeighbors(lNW,true);
        incEqualSizeNeighbors(lNE,true);
    }

}

void quadTree::approximateEmptyLeaves()
{
    std::deque<quadLeaf*>::iterator it = listLeaf.begin();
    while (it != listLeaf.end())
    {
        if((*it)->isEmpty()){
            (*it)->fillWithNeighbors(pts);
        }
        it++;
    }
}

quadLeaf* quadTree::findFirstLeafAboveThreshold()
{
    //std::cout<<"in findFirstLeafAboveThreshold listLeaf size "<<listLeaf.size()<<std::endl;
    std::deque<quadLeaf*>::iterator it = listLeaf.begin();
    while (it != listLeaf.end())
    {
        if(!(*it)->isEmpty()){
        //if((*it)->getLocalCell()->getError() > errorThreshold  && !(*it)->notToDivide)
        if( !(*it)->notToDivide)
        {
            return *it;
        }}
        it++;
    }
    //std::cout<<"return NULL"<<std::endl;
    return NULL;
}

bool quadTree::isLeafAboveThreshold()
{
    //std::cout<<"in isLeafAboveThreshold"<<std::endl;
    std::deque<quadLeaf*>::iterator it = listLeaf.begin();
    while (it != listLeaf.end())
    {
        if(!(*it)->isEmpty()){
        //if((*it)->getLocalCell()->getError() > errorThreshold  && !(*it)->notToDivide)
        if(!(*it)->notToDivide)
        {
            //std::cout<<"isLeafAboveThreshold true"<<std::endl;
            return true;
        }}
        it++;
    }
    //std::cout<<"isLeafAboveThreshold false"<<std::endl;
    return false;
}

quadLeaf* quadTree::findByBitset(std::bitset<QUAD_MAX_LEVEL> pCode, int pLevel)
{
    std::deque<quadLeaf*>::iterator it = listLeaf.begin();
    while (it != listLeaf.end())
    {
        if((*it)->getLocationCode() == pCode && (*it)->getLevel() == pLevel)
        {
            return *it;
        }
        it++;
    }
    return NULL;
}

int quadTree::getPositionInList(quadLeaf leaf)
{
    std::deque<quadLeaf*>::iterator it = std::find_if(listLeaf.begin(), listLeaf.end(),findByCodeWithChildren(leaf.getLocationCode(),leaf.getLevel()));
    return std::distance( listLeaf.begin(), it );
}

void quadTree::incEqualSizeNeighbors(quadLeaf* pLeaf, bool checkIfBrother)
{
    if(pLeaf->deltaLevelEast!=INT_MAX){
    std::bitset<QUAD_MAX_LEVEL> lcEast = calcNeighborsEqualSize(pLeaf->getLocationCode(),EAST_NEIGHBOR);
    if(findByBitset(lcEast,pLeaf->getLevel()))
    {
        if(checkIfBrother)
        {
            if(!isBrother(pLeaf->getLocationCode(),lcEast))
            {
                if(findByBitset(lcEast,pLeaf->getLevel())->deltaLevelWest != INT_MAX)
                {
                    findByBitset(lcEast,pLeaf->getLevel())->deltaLevelWest++;
                }
            }

        }else{
            if(findByBitset(lcEast,pLeaf->getLevel())->deltaLevelWest != INT_MAX)
            {
                findByBitset(lcEast,pLeaf->getLevel())->deltaLevelWest++;
            }
        }
    }
    }

    if(pLeaf->deltaLevelNorth!=INT_MAX){
    std::bitset<QUAD_MAX_LEVEL> lcNorth = calcNeighborsEqualSize(pLeaf->getLocationCode(),NORTH_NEIGHBOR);
    if(findByBitset(lcNorth,pLeaf->getLevel()))
    {
        if(checkIfBrother)
        {
            if(!isBrother(pLeaf->getLocationCode(),lcNorth))
            {
                if(findByBitset(lcNorth,pLeaf->getLevel())->deltaLevelSouth != INT_MAX)
                {
                    findByBitset(lcNorth,pLeaf->getLevel())->deltaLevelSouth++;
                }
            }
        }else{
            if(findByBitset(lcNorth,pLeaf->getLevel())->deltaLevelSouth != INT_MAX)
            {
                findByBitset(lcNorth,pLeaf->getLevel())->deltaLevelSouth++;
            }
        }
    }
    }

    if(pLeaf->deltaLevelWest!=INT_MAX){
    std::bitset<QUAD_MAX_LEVEL> lcWest = calcNeighborsEqualSize(pLeaf->getLocationCode(),WEST_NEIGHBOR);
    if(findByBitset(lcWest,pLeaf->getLevel()))
    {
        if(checkIfBrother)
        {
            if(!isBrother(pLeaf->getLocationCode(),lcWest))
            {
                if(findByBitset(lcWest,pLeaf->getLevel())->deltaLevelEast != INT_MAX)
                {
                    findByBitset(lcWest,pLeaf->getLevel())->deltaLevelEast++;
                }
            }
        }else{
            if(findByBitset(lcWest,pLeaf->getLevel())->deltaLevelEast != INT_MAX)
            {
                findByBitset(lcWest,pLeaf->getLevel())->deltaLevelEast++;
            }
        }
    }
    }

    if(pLeaf->deltaLevelSouth!=INT_MAX){
    std::bitset<QUAD_MAX_LEVEL> lcSouth = calcNeighborsEqualSize(pLeaf->getLocationCode(),SOUTH_NEIGHBOR);
    if(findByBitset(lcSouth,pLeaf->getLevel()))
    {
        if(checkIfBrother)
        {
            if(!isBrother(pLeaf->getLocationCode(),lcSouth))
            {
                if(findByBitset(lcSouth,pLeaf->getLevel())->deltaLevelNorth != INT_MAX)
                {
                    findByBitset(lcSouth,pLeaf->getLevel())->deltaLevelNorth++;
                }
            }
        }else{
            if(findByBitset(lcSouth,pLeaf->getLevel())->deltaLevelNorth != INT_MAX)
            {
                findByBitset(lcSouth,pLeaf->getLevel())->deltaLevelNorth++;
            }
        }
    }
    }
}

std::bitset<QUAD_MAX_LEVEL> quadTree::calcPosition(std::bitset<QUAD_MAX_LEVEL> pLocation, int pPosition )
{
    std::bitset<QUAD_MAX_LEVEL> bsPosition( (long) pPosition );
    return bsPosition|(pLocation<<2);
}

std::bitset<QUAD_MAX_LEVEL> quadTree::calcNeighborsEqualSize(std::bitset<QUAD_MAX_LEVEL> pLocation, std::bitset<QUAD_MAX_LEVEL> pDirection )
{
    return quadLocationAdd(pLocation,pDirection);
}

std::bitset<QUAD_MAX_LEVEL> quadTree::quadLocationAdd( std::bitset<QUAD_MAX_LEVEL> a, std::bitset<QUAD_MAX_LEVEL> b ) {

    std::bitset<QUAD_MAX_LEVEL> bstx = getTx(quadLeaf::levelMax);
    std::bitset<QUAD_MAX_LEVEL> bsty = getTy(quadLeaf::levelMax);
    return (add(a|bsty,b&bstx)&bstx)|(add(a|bstx,b&bsty)&bsty);
}

std::bitset<QUAD_MAX_LEVEL> quadTree::add(std::bitset<QUAD_MAX_LEVEL> a, std::bitset<QUAD_MAX_LEVEL> b)
{
    std::bitset<QUAD_MAX_LEVEL> const m((long)1);
    std::bitset<QUAD_MAX_LEVEL> result;
    for (std::size_t i = 0; i < result.size(); ++i) {
        std::bitset<QUAD_MAX_LEVEL> const diff(((a >> i)&m).to_ulong() + ((b >> i)&m).to_ulong() + (result >> i).to_ulong());
        result ^= (diff ^ (result >> i)) << i;
    }
    return result;
}

std::bitset<QUAD_MAX_LEVEL> quadTree::getTx(int pLevelMax)
{
    std::bitset<QUAD_MAX_LEVEL> ret;
    std::bitset<QUAD_MAX_LEVEL> unit( (long) 1);
    for(int i=0;i<pLevelMax;i++)
    {
        ret = (ret << 2) | unit;
    }
    return ret;
}

std::bitset<QUAD_MAX_LEVEL> quadTree::getTy(int pLevelMax)
{
    std::bitset<QUAD_MAX_LEVEL> ret;
    std::bitset<QUAD_MAX_LEVEL> unit( (long) 2 );
    for(int i=0;i<pLevelMax;i++)
    {
        ret = (ret << 2) | unit;
    }
    return ret;
}

bool quadTree::isBrother(std::bitset<QUAD_MAX_LEVEL> pLeafA, std::bitset<QUAD_MAX_LEVEL> pLeafB)
{
    bool test=true;
    for(std::size_t i=2;i<pLeafA.size();i++)
    {
        if(pLeafA.test(i) != pLeafB.test(i))test=false;
    }
    return test;
}

void quadTree::buildLeafsSurface()
{
    for(std::deque<quadLeaf*>::iterator it = listLeaf.begin();it!=listLeaf.end();it++)
    {
        if(!(*it)->isEmpty()){
          (*it)->polygonizeLevelSet((*it)->getQuadrant(),min.z,max.z,10,10,10,0.);
        }
    }
}

void quadTree::printList()
{
    std::cout<<"***************************************"<<std::endl;
    for(std::size_t i=0;i<listLeaf.size();i++)
    {
        if(listLeaf.at(i)->getParent()!=NULL)
        {
            std::cout<<i<<" "<<listLeaf.at(i)->getLocationCode()<<" empty "<<listLeaf.at(i)->isEmpty()<<" lvl "<<listLeaf.at(i)->getLevel()<<" son of "<<listLeaf.at(i)->getParent()->getLocationCode()<<" lvl "<<listLeaf.at(i)->getParent()->getLevel()<<std::endl;
        }else{
            std::cout<<i<<" "<<listLeaf.at(i)->getLocationCode()<<" son of NULL"<<std::endl;
        }
    }
}
