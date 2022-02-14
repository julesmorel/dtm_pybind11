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

#ifndef HASHMAPDATA
#define HASHMAPDATA

#include <unordered_map>

#include "boost/functional/hash.hpp"

class key{
public:
    key(int ix, int id) : index(ix),id(id){}
    int index;
    int id;
};

struct hashFct{
    size_t operator()(const key &k) const{
    size_t seed = 0;
    boost::hash_combine(seed,boost::hash_value(k.index));
    boost::hash_combine(seed,boost::hash_value(k.id));
    return seed;
    }
};

struct equalsFct{
  bool operator()( const key& lhs, const key& rhs ) const{
    return (lhs.index == rhs.index) && (lhs.id == rhs.id);
  }
};

typedef std::unordered_map<key, float, hashFct, equalsFct> hashMapScalarField;

#endif // HASHMAPDATA

