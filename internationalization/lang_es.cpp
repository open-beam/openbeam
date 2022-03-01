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

#include "localization.h"

/** String tables for en (English) language - Strings MUST BE in the same
 *  order than declared in strings.h
 * Make sure this file is encoded in UTF-8
 */
const char* openbeam::localization::strs_es[] = {
    "Restricciones",  // STR_Constraints
    "restricción",  // STR_constraint
    "Cargas",  // STR_Loads
    "carga",  // STR_load
    "Desplazamientos",  // STR_Displacements
    "desplazamiento",  // STR_displacement
    "Reacciones",  // STR_Reactions
    "reacción",  // STR_reaction
    "GDL",  // STR_dof
    "nodo",  // STR_node
    "y",  // STR_and
    "Pulsar para ampliar",  // STR_click_to_enlarge
    "Matriz de rigidez global",  // STR_GlobalStiffnessMatrix
    "Matrices de rigidez de todos los elementos",  // STR_AllElementsStiffnessMatrices
    "Barra",  // STR_Bar
};
