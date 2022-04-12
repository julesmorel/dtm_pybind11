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

#ifndef ISOSURFACE2_H
#define ISOSURFACE2_H

#include "scalarfield.h"

#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct triangle {
    unsigned int pointIndex[3];
};

class isoSurface
{
public:

    isoSurface(){}
    isoSurface(scalarField * field , float pIsoLevel, unsigned int pnbCellsX, unsigned int pnbCellsY, unsigned int pnbCellsZ, float pCellLengthX, float pCellLengthY, float pCellLengthZ);

    void generateSurface();

    const std::map<unsigned int, pcl::PointXYZ>& getMapPoints() const {return mapPoints;}
    const std::vector<triangle>& getTriangles() const {return listTriangles;}

    const pcl::PointXYZ& getBBMin(){return bbMin;}
    const pcl::PointXYZ& getBBMax(){return bbMax;}

    double getVolumeInside() const {return volumeInsideMesh;}
    double getVolumeIntersected() const {return volumeIntersectedByMesh;}
    double getVolume() const {return volumeInsideMesh+0.5*volumeIntersectedByMesh;}

    scalarField * getField() const {return field;}

    void saveMeshAsPly(std::string filename);
    void saveMeshVertices(std::string filename);

private:

    scalarField * field;

    std::map<unsigned int, pcl::PointXYZ> mapPoints;
    std::vector<triangle> listTriangles;

    // No. of cells in x, y, and z directions.
    unsigned int nbCellsX, nbCellsY, nbCellsZ;

    // Cell length in x, y, and z directions.
    float cellLengthX, cellLengthY, cellLengthZ;

    // The isosurface value.
    float isoLevel;

    pcl::PointXYZ min;

    pcl::PointXYZ bbMin, bbMax;

    //flag for volume computation
    bool computeVolume;
    double volumeInsideMesh;
    double volumeIntersectedByMesh;

    // Lookup tables used in the construction of the isosurface.
    static const unsigned int edgeTable[256];
    static const int triTable[256][16];

    void addPointToList(int x, int y, int z, int index);
    unsigned int getEdgeID(unsigned int nX, unsigned int nY, unsigned int nZ, unsigned int nEdgeNo);
    unsigned int getVertexID(unsigned int nX, unsigned int nY, unsigned int nZ);
    pcl::PointXYZ calculateIntersection(unsigned int nX, unsigned int nY, unsigned int nZ, unsigned int nEdgeNo);
    pcl::PointXYZ interpolate(float fX1, float fY1, float fZ1, float fX2, float fY2, float fZ2, float tVal1, float tVal2);
    pcl::PointXYZ getPointByRectangle(float v1x, float v1y, float v1z);

    void removeDuplicatePoints();
};

#endif // ISOSURFACE2_H
