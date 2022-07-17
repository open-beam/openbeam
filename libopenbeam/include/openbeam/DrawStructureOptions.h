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

#pragma once

#include <openbeam/types.h>

namespace openbeam
{
struct StaticSolveProblemInfo;

/** Structure for holding all the parameters for
 * CFiniteElementProblem::saveAsImageSVG */
struct DrawStructureOptions
{
    DrawStructureOptions()  = default;
    ~DrawStructureOptions() = default;

    bool show_nodes_original    = true;
    bool show_nodes_deformed    = false;
    bool show_node_labels       = false;
    bool show_elements_original = true;  //!< Draw original undeformed elements
    bool show_elements_deformed = false;  //!< Draw final deformed elements
    bool show_element_labels    = true;
    bool show_loads             = true;
    bool show_constraints       = true;

    /// Default: 1024px (Height is automatically determined)
    double image_width = 1024;
    double node_radius = 2e-2;

    double       EDGE_WIDTH         = 5e-3;
    double       BEAM_PINNED_RADIUS = 10e-3;
    unsigned int PIN_SPHERE_DIVS    = 8;  // long and latitude sphere divisions
    unsigned int NODE_SPHERE_DIVS   = 5;  // long and latitude sphere divisions

    /// Scale of deformed states (Default=0 means autodetermination from \a
    /// deformed_scale_auto_max_image_ratio)
    double deformed_scale_factor = 0;

    double deformed_scale_factor_for_bbox = 0;

    /// Used to auto determinate \a deformed_scale_factor: the ratio of the
    /// image size that will equal the maximum displacement of a deformed state.
    /// Default=0.1 (10%)
    double deformed_scale_auto_max_image_ratio = 0.1;

    /// Maximum size (in ratio wrt the largest structure dimension) of the
    /// largest load on a node. Default: 0.1 (10%)
    double node_loads_max_relative_size = 0.1;

    double labels_size = 15.0;

    /// In structure units.
    double margin_left = 0, margin_right = 0, margin_top = 0, margin_bottom = 0;

    //--- color params ---
    double elements_original_alpha = 1, elements_deformed_alpha = 1;  //!< [0,1]
    double nodes_original_alpha = 1, nodes_deformed_alpha = 1;  //!< [0,1]
    double constraints_original_alpha = 1,
           constraints_deformed_alpha = 1;  //!< [0,1]
    double loads_original_alpha = 1, loads_deformed_alpha = 1;  //!< [0,1]

};  // end of DrawStructureOptions

struct DrawElementExtraParams
{
    DrawElementExtraParams() = default;

    /// If false, draw the final deformed position.
    bool                          draw_original_position = true;
    size_t                        element_index          = 0;
    double                        color_alpha            = 1;
    const StaticSolveProblemInfo* solver_info            = nullptr;
    num_t                         deformed_scale_factor  = 1;
};

struct RenderInitData
{
    num_t  min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    double width = 0, height = 0;
    double scaleFactor = 1;
};

}  // namespace openbeam
