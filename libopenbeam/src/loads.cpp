/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2021  Jose Luis Blanco Claraco                       |
   |                              University of Malaga                         |
   |                                                                           |
   | OpenBeam is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   | OpenBeam is distributed in the hope that it will be useful,               |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with OpenBeam.  If not, see <http://www.gnu.org/licenses/>.     |
   |                                                                           |
   +---------------------------------------------------------------------------+
 */

#include <openbeam/loads.h>

using namespace std;
using namespace openbeam;

OB_TODO("It seems to be a bug with simple shafts with torsional loads")

/** Class factory from element name, or nullptr for an unknown element:
 *  Element names:
 *		- "TEMPERATURE": CLoadConstTemperature
 *		- "DISTRIB_UNIFORM": CLoadDistributedUniform
 *		- "CONCENTRATED": CLoadConcentratedForce
 *		- "TRIANGULAR": CLoadDistributedTriangular
 */
CLoadOnBeam::Ptr CLoadOnBeam::createLoadByName(const std::string& s)
{
    if (strCmpI("TEMPERATURE", s))
        return std::make_shared<CLoadConstTemperature>();
    if (strCmpI("DISTRIB_UNIFORM", s))
        return std::make_shared<CLoadDistributedUniform>();
    if (strCmpI("CONCENTRATED", s))
        return std::make_shared<CLoadConcentratedForce>();
    if (strCmpI("TRIANGULAR", s))
        return std::make_shared<CLoadDistributedTriangular>();

    return nullptr;
}
