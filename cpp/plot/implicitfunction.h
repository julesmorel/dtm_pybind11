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

#ifndef IMPLICITFUNCTION_H
#define IMPLICITFUNCTION_H

#include "isosurface.h"
#include "../core/rectangle.h"

class implicitFunction
{
public:

    implicitFunction(){}
    implicitFunction(pcl::PointXYZ min, pcl::PointXYZ max){BBMin=min; BBMax=max;}
    ~implicitFunction(){}

    const pcl::PointXYZ& getBBmin(){return BBMin;}
    const pcl::PointXYZ& getBBmax(){return BBMax;}

    virtual void fillScalarField(scalarField &field){}

    void polygonizeLevelSet(rectangle prect, float zmin, float zmax, int nx, int ny, int nz, float isoLevel=0.);
    const isoSurface& getLevelSet() const {return surface;}
    void writePly(std::string file);
    void writeScalarField(std::string file);
    void writeAscii(std::string file);

protected:

    pcl::PointXYZ BBMin, BBMax;
    isoSurface surface;
};

#endif // IMPLICITFUNCTION_H
