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

#ifndef SCALARFIELD_H
#define SCALARFIELD_H

#include <vector>
#include <float.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "hashmapdata.h"
#include "../core/rectangle.h"

class scalarField
{
  public:

    scalarField(pcl::PointXYZ bbMin, pcl::PointXYZ bbMax, int x, int y, int z);
    scalarField(rectangle prect, float pzmin, float pzmax, int x, int y, int z);
    ~scalarField();

    double getValue(int i, int j, int k, int id=0);//{return data[k*xlen*ylen+j*xlen+i];}
    double getValueByIndex(int ix, int id);
    void setValue(int i,int j, int k, double pVal, int id=0);//{ data[k*xlen*ylen+j*xlen+i] = pVal;}

    static const float NODATA;

    pcl::PointXYZ& getBBmin(){return BBMin;}
    pcl::PointXYZ& getBBmax(){return BBMax;}

    int getXNode(){return xlen;}
    int getYNode(){return ylen;}
    int getZNode(){return zlen;}

    float getSizeCellX(){return sizeCellX;}
    float getSizeCellY(){return sizeCellY;}
    float getSizeCellZ(){return sizeCellZ;}

    rectangle getRectangle(){return rect;}
    float getZmin(){return zmin;}
    float getZmax(){return zmax;}

    void printToFile(std::string filename);

    void reset();

    pcl::PointXYZ getPointFromIndex(int i, int j, int k);

    int getSizeMap(){return map.size();}

    void mergeFields(int numberFields);

    bool isInMap(int i, int j, int k, int id){return map.find(key(indexAtXYZ(i,j,k),id))!=map.end();}

     std::vector<int>& getListIndex(){return listIndex;}

  private:

    rectangle rect;
    float zmin,zmax;

    float *data;

    hashMapScalarField map;

    std::vector<int> listIndex;

    int xlen, ylen, zlen;
    float sizeCellX,sizeCellY,sizeCellZ;

    pcl::PointXYZ BBMin, BBMax;

    int indexAtXYZ(int i,int j, int k);
};

#endif // SCALARFIELD_H
