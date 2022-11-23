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

#include <openbeam/DrawStructureOptions.h>

using namespace openbeam;

void DrawStructureOptions::loadFromYaml(const mrpt::containers::yaml& d)
{
    MCP_LOAD_OPT(d, show_nodes_original);
    MCP_LOAD_OPT(d, show_nodes_deformed);
    MCP_LOAD_OPT(d, show_node_labels);
    MCP_LOAD_OPT(d, show_elements_original);
    MCP_LOAD_OPT(d, show_elements_deformed);
    MCP_LOAD_OPT(d, show_element_labels);
    MCP_LOAD_OPT(d, show_loads);
    MCP_LOAD_OPT(d, show_constraints);
    MCP_LOAD_OPT(d, deformed_scale_factor);
    MCP_LOAD_OPT(d, deformed_scale_factor_for_bbox);
    MCP_LOAD_OPT(d, elements_original_alpha);

    MCP_LOAD_OPT(d, show_force_axial);
    MCP_LOAD_OPT(d, show_force_shear_y);
    MCP_LOAD_OPT(d, show_bending_moment_z);
    MCP_LOAD_OPT(d, show_force_shear_z);
    MCP_LOAD_OPT(d, show_bending_moment_y);
    MCP_LOAD_OPT(d, show_torsion_moment);
}
