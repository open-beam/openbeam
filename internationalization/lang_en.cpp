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

#include "localization.h"

/** String tables for en (English) language - Strings MUST BE in the same
 *  order than declared in strings.h
 * Make sure this file is encoded in UTF-8
 */
const char* openbeam::localization::strs_en[] = {
    "Constraints",  // STR_Constraints
    "constraint",  // STR_constraint
    "Loads",  // STR_Loads
    "load",  // STR_load
    "Displacements",  // STR_Displacements
    "displacement",  // STR_displacement
    "Reactions",  // STR_Reactions
    "reaction",  // STR_reaction
    "DOF",  // STR_dof
    "node",  // STR_node
    "and",  // STR_and
    "Click to enlarge",  // STR_click_to_enlarge
    "Global stiffness matrix",  // STR_GlobalStiffnessMatrix
    "All elements stiffness matrices",  // STR_AllElementsStiffnessMatrices
    "Bar",  // STR_Bar
};
