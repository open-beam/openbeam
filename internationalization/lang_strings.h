/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2013  Jose Luis Blanco Claraco                       |
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

#pragma once

namespace openbeam
{
namespace localization
{
enum TStringID
{
    STR_Constraints = 0,
    STR_constraint,
    STR_Loads,
    STR_load,
    STR_Displacements,
    STR_displacement,
    STR_Reactions,
    STR_reaction,
    STR_dof,
    STR_node,
    STR_and,
    STR_click_to_enlarge,
    STR_GlobalStiffnessMatrix,
    STR_AllElementsStiffnessMatrices,
    STR_Bar,

    // Always leave this as an END flag:
    STR_NUMBER_OF_IDS
};
}
}  // namespace openbeam
