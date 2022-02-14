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

#include <pcl/common/geometry.h>
#include "scalarfield.h"
#include <fstream>

const float scalarField::NODATA = FLT_MAX;

scalarField::scalarField(pcl::PointXYZ bbMin, pcl::PointXYZ bbMax, int x, int y, int z)
{
    BBMin=bbMin;
    BBMax=bbMax;

    xlen=x;
    ylen=y;
    zlen=z;

    sizeCellX = ((BBMax.x-BBMin.x))/(float)(xlen-1);
    sizeCellY = ((BBMax.y-BBMin.y))/(float)(ylen-1);
    sizeCellZ = ((BBMax.z-BBMin.z))/(float)(zlen-1);
}

scalarField::scalarField(rectangle prect, float pzmin, float pzmax, int x, int y, int z): rect(prect)
{
  xlen=x+1;
  ylen=y+1;
  zlen=z+1;

  float dx = pcl::geometry::distance(prect.getP0(),prect.getP1());
  float dy = pcl::geometry::distance(prect.getP0(),prect.getP3());
  zmax=pzmax;
  zmin=pzmin;

  sizeCellX = dx/(float)(xlen);
  sizeCellY = dy/(float)(ylen);
  sizeCellZ = ((zmax-zmin))/(float)(zlen);
}

double scalarField::getValue(int i,int j, int k, int id)
{
    if(map.find(key(indexAtXYZ(i,j,k),id))!=map.end())
    {
        return map[key(indexAtXYZ(i,j,k),id)];
    }else{
        return scalarField::NODATA;
    }
}

double scalarField::getValueByIndex(int ix, int id)
{
    if(map.find(key(ix,id))!=map.end())
    {
        return map[key(ix,id)];
    }else{
        return scalarField::NODATA;
    }
}

void scalarField::setValue(int i,int j, int k, double pVal, int id)
{
    int ix = indexAtXYZ(i,j,k);
    listIndex.push_back(ix);
    map[key(indexAtXYZ(i,j,k),id)]=pVal;
}

int scalarField::indexAtXYZ(int i,int j, int k){
    return k*xlen*ylen+j*xlen+i;
}

scalarField::~scalarField(){}

void scalarField::printToFile(std::string filename)
{
    float dx = pcl::geometry::distance(rect.getP0(),rect.getP1());
    float dy = pcl::geometry::distance(rect.getP0(),rect.getP3());

    std::ofstream fileStream(filename.c_str(), std::ios::out | std::ios::trunc);

    if(fileStream)
    {
        for ( int i = 0 ; i < xlen ; i++ )
        {
            for ( int j = 0 ; j < ylen ; j++ )
            {
                for ( int k = 0 ; k < zlen ; k++ )
                {
                    pcl::PointXYZ p = getPointFromIndex(i,j,k);

                    float val = getValue(i,j,k);
                    if(val != scalarField::NODATA)
                    {
                        fileStream<<p.x<<" "<<p.y<<" "<<p.z<<" "<<val<<std::endl;
                    }
                }

            }
        }
        fileStream.close();
    }
}

void scalarField::reset()
{
    map.clear();
}

pcl::PointXYZ scalarField::getPointFromIndex(int i, int j, int k)
{
  float rx = (float)i/(float)(xlen-1);
  float ry = (float)j/(float)(ylen-1);
  float rz = (float)k/(float)(zlen-1);

  pcl::PointXYZ p = rect.getPointWithRatio(rx, ry);
  p.z = zmin + (zmax-zmin)*rz;

  return p;
}

void scalarField::mergeFields(int numberFields)
{

    for ( int p = 0 ; p < listIndex.size() ; p++ )
    {
        int ix = listIndex.at(p);

        for(int n=1;n<numberFields;n++){

            float newVal = getValueByIndex(ix,n);
            if(newVal != scalarField::NODATA)
            {
                float oldVal = getValueByIndex(ix,0);
                if(oldVal == scalarField::NODATA)
                {
                    map[key(ix,0)] = newVal;
                }else{
                    if(oldVal > 0){
                        if(newVal<0.){
                            map[key(ix,0)] = newVal;
                        }else{
                            map[key(ix,0)] = newVal + oldVal;
                        }
                    }
                }
            }
        }
    }
}
