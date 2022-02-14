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
#include "wendlandRbf.h"

#include <cmath>

double wendlandRbf::getValueRbfWendland(double r)
{
    double val=0.;
    if(r<1.) val = (1-r)*(1-r)*(1-r)*(1-r)*(1.+4.*r);
    //if(r<1.) val = (1-r)*(1-r)*(1-r)*(1-r)*(15.*r*r+12.*r+3);
    return val;
}

double wendlandRbf::getValueRbfWendlandDeriv(double r)
{
    double val=0.;
    if(r<1.) val = (1-r)*(1-r)*(1-r)*(-20.*r);
    return val;
}

double wendlandRbf::getValueRbfWendlandZero(double r)
{
    double val=0.;
    if(r<1.) val=(1-r);
    return val;
}

double wendlandRbf::getValueRbfWendland8(double r)
{
    double val=0.;
    if(r<1.) val = (1-r)*(1-r)*(1-r)*(1-r)*(1-r)*(1-r)*(1-r)*(1-r)*(1.+8.*r+25.*r*r+32.*r*r*r);
    return val;
}
