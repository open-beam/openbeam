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

#include "types.h"

namespace openbeam
{
struct TStaticSolveProblemInfo;

/** Structure for holding all the parameters for
 * CFiniteElementProblem::saveAsImageSVG */
struct TDrawStructureOptions
{
    // --- bool switches ----
    bool show_nodes_original, show_nodes_deformed, show_node_labels;
    bool show_elements_original;  //!< Draw original undeformed elements
    bool show_elements_deformed;  //!< Draw final deformed elements
    bool show_element_labels;
    bool show_loads;
    bool show_constraints;

    //--- real params ---
    double
           image_width;  //!< Default: 1024px (Height is automatically determined)
    double node_radius;  //!< Default: 5e-2;
    double deformed_scale_factor;  //!< Scale of deformed states (Default=0 ->
                                   //!< means autodetermination from \a
                                   //!< deformed_scale_auto_max_image_ratio)
    double deformed_scale_factor_for_bbox;
    double
        deformed_scale_auto_max_image_ratio;  //!< Used to auto determinate \a
                                              //!< deformed_scale_factor: the
                                              //!< ratio of the image size that
                                              //!< will equal the maximum
                                              //!< displacement of a deformed
                                              //!< state. Default=0.1 (10%)
    double node_loads_max_relative_size;  //!< Maximum size (in ratio wrt the
                                          //!< largest structure dimension) of
                                          //!< the largest load on a node.
                                          //!< Default: 0.1 (10%)
    double labels_size;  //!< Default: 15
    double margin_left, margin_right, margin_top,
        margin_bottom;  //!< (Default=-1.0) In structure units. <0: auto

    //--- color params ---
    double elements_original_alpha, elements_deformed_alpha;  //!< [0,1]
    double nodes_original_alpha, nodes_deformed_alpha;  //!< [0,1]
    double constraints_original_alpha, constraints_deformed_alpha;  //!< [0,1]
    double loads_original_alpha, loads_deformed_alpha;  //!< [0,1]

    TDrawStructureOptions()
        :  // --- bool switches ----
          show_nodes_original(true),
          show_nodes_deformed(false),
          show_node_labels(false),
          show_elements_original(true),
          show_elements_deformed(false),
          show_element_labels(true),
          show_loads(true),
          show_constraints(true),
          //--- real params ---
          image_width(1024),
          node_radius(2e-2),
          deformed_scale_factor(0),
          deformed_scale_factor_for_bbox(0),
          deformed_scale_auto_max_image_ratio(0.1),
          node_loads_max_relative_size(0.1),
          labels_size(15.0),
          margin_left(0),
          margin_right(0),
          margin_top(0),
          margin_bottom(0),
          //--- color params ---
          elements_original_alpha(1),
          elements_deformed_alpha(1),
          nodes_original_alpha(1),
          nodes_deformed_alpha(1),
          constraints_original_alpha(1),
          constraints_deformed_alpha(1),
          loads_original_alpha(1),
          loads_deformed_alpha(1)
    {
    }
};  // end of TDrawStructureOptions

struct TDrawElementExtraParams
{
    TDrawElementExtraParams()
        : draw_original_position(true),
          color_alpha(1),
          solver_info(NULL),
          deformed_scale_factor(1)
    {
    }

    bool draw_original_position;  //!< If false, draw the final deformed
                                  //!< position.
    size_t                         element_index;
    double                         color_alpha;
    const TStaticSolveProblemInfo* solver_info;
    num_t                          deformed_scale_factor;
};

struct TRenderInitData
{
    num_t  min_x, max_x, min_y, max_y;
    double width, height;
    double scaleFactor;
};

}  // namespace openbeam
