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

#include "implicitfunction.h"
#include <pcl/common/geometry.h>


void implicitFunction::polygonizeLevelSet(rectangle prect, float pzmin, float pzmax, int nx, int ny, int nz, float isoLevel)
{
  float dx = pcl::geometry::distance(prect.getP0(),prect.getP1());
  float dy = pcl::geometry::distance(prect.getP0(),prect.getP3());
  float zmax=pzmax;
  float zmin=pzmin;

  float sizeCellX = dx/(float)nx;
  float sizeCellY = dy/(float)ny;
  float sizeCellZ = ((zmax-zmin))/(float)nz;

  clock_t t = clock();
  //prect.print();
  scalarField field(prect,zmin,zmax,nx,ny,nz);
  fillScalarField(field);

  //field.printToFile("scalarField.xyz");

  t = clock() - t;
  //std::cout<<"        Scalar field in "<<((float)t)/CLOCKS_PER_SEC<<" s"<<std::endl;
  surface = isoSurface(&field, isoLevel,nx,ny,nz,sizeCellX,sizeCellY,sizeCellZ);
  surface.generateSurface();
  t = clock() - t;
  //std::cout<<"        Levelset extraction in "<<((float)t)/CLOCKS_PER_SEC<<" s"<<std::endl;
}

void implicitFunction::writePly(std::string file)
{
    surface.saveMeshAsPly(file);
}

void implicitFunction::writeAscii(std::string file)
{
    surface.saveMeshVertices(file);
}

void implicitFunction::writeScalarField(std::string file)
{
    surface.getField()->printToFile(file);
}
