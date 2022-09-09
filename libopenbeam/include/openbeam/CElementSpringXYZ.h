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

#include "CElement.h"
#include "types.h"

namespace openbeam
{
/** A spring element with three elastic components in the local X, Y and Z
 * directions between two given nodes.
 */
class CElementSpringXYZ : public CElement
{
   public:
    CElementSpringXYZ();
    CElementSpringXYZ(
        const size_t from_node_id, const size_t to_node_id, const num_t Kx,
        const num_t Ky, const num_t Kz);

    /** Return the stiffness submatrices between each pair of edges in this
     * element, for the current element state.
     */
    void getLocalStiffnessMatrices(
        std::vector<TStiffnessSubmatrix>& outSubMats) const override;

    void getLocalDoFs(std::vector<used_DoFs_t>& dofs) const override;

    num_t Kx, Ky, Kz;  //!< Stiffness constants of the spring (N/m)

    /** Parse a set of parameters by (casi insensitive) name and set the element
     * values from them.
     *  Each element must document the supported parameters and their meaning.
     */
    void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx) override;

    /** Draws the element to a SVG Cairo context (a pointer to a
     * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
     * options */
    void drawSVG(
        void* _cairo_context, const DrawStructureOptions& options,
        const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo* meshing_info) const override;
    mrpt::opengl::CSetOfObjects::Ptr getVisualization(
        const DrawStructureOptions&   options,
        const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo*         meshing_info) const override
    {
        return {};
    }

    /** Mesh this element into a set of (possibly) smaller ones */
    void do_mesh(
        const size_t my_idx, CStructureProblem& out_fem,
        MeshOutputInfo& out_info, const MeshParams& params) override;
};
}  // namespace openbeam
