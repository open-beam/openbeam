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

#include <openbeam/CElementTorsionSpring.h>
#include <openbeam/CStructureProblem.h>

using namespace std;
using namespace openbeam;

CElementTorsionSpring::CElementTorsionSpring()
    : CElement(2), K(UNINITIALIZED_VALUE)
{
}

CElementTorsionSpring::CElementTorsionSpring(
    const size_t from_node_id, const size_t to_node_id, const num_t K)
    : CElement(2, from_node_id, to_node_id), K(K)
{
}

/** Return the stiffness submatrices between each pair of edges in this element,
 * for the current element state.
 */
void CElementTorsionSpring::getLocalStiffnessMatrices(
    std::vector<TStiffnessSubmatrix>& outSubMats) const
{
    outSubMats.resize(3);
    TStiffnessSubmatrix& K11 = outSubMats[0];
    TStiffnessSubmatrix& K22 = outSubMats[1];
    TStiffnessSubmatrix& K12 = outSubMats[2];

    if (K == UNINITIALIZED_VALUE)
        throw std::runtime_error(format(
            "CElementTorsionSpring: Uninitialized parameter K (Element in "
            "%u->%u)",
            static_cast<unsigned int>(this->conected_nodes_ids[0]),
            static_cast<unsigned int>(this->conected_nodes_ids[1])));

    // k11 --------------
    K11.edge_in      = 0;
    K11.edge_out     = 0;
    K11.matrix       = Matrix66::Zero();
    K11.matrix(5, 5) = K;

    // k22 --------------
    K22.edge_in  = 1;
    K22.edge_out = 1;
    K22.matrix   = K11.matrix;

    // k12 --------------
    K12.edge_in      = 0;
    K12.edge_out     = 1;
    K12.matrix       = Matrix66::Zero();
    K12.matrix(5, 5) = -K;
}

void CElementTorsionSpring::getLocalDoFs(std::vector<used_DoFs_t>& dofs) const
{
    dofs.resize(getNumberEdges());

    static const used_DoFs_t sDofs = {false, false, false, false, false, true};
    dofs[0] = dofs[1] = sDofs;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CElementTorsionSpring::loadParamsFromSet(
    const mrpt::containers::yaml& p, const EvaluationContext& ctx)
{
    if (p.has("K")) this->K = ctx.evaluate(p["K"]);
}

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CElementTorsionSpring::drawSVG(
    void* _cairo_context, const DrawStructureOptions& options,
    const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    OB_TODO("Implement SVG")
#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CElementTorsionSpring::do_mesh(
    const size_t my_idx, CStructureProblem& out_fem, MeshOutputInfo& out_info,
    const MeshParams& params)
{
    throw std::runtime_error("TO DO");
}
