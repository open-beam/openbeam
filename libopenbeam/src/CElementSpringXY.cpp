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

#include <openbeam/CElementSpringXY.h>
#include <openbeam/CStructureProblem.h>

using namespace std;
using namespace openbeam;

CElementSpringXY::CElementSpringXY() : CElement(2), Kx(0), Ky(0) {}

CElementSpringXY::CElementSpringXY(
    const size_t from_node_id, const size_t to_node_id, const num_t Kx,
    const num_t Ky)
    : CElement(2, from_node_id, to_node_id), Kx(Kx), Ky(Ky)
{
}

/** Return the stiffness submatrices between each pair of edges in this element,
 * for the current element state.
 */
void CElementSpringXY::getLocalStiffnessMatrices(
    std::vector<TStiffnessSubmatrix>& outSubMats) const
{
    outSubMats.resize(3);
    TStiffnessSubmatrix& K11 = outSubMats[0];
    TStiffnessSubmatrix& K22 = outSubMats[1];
    TStiffnessSubmatrix& K12 = outSubMats[2];

    // k11 --------------
    K11.edge_in      = 0;
    K11.edge_out     = 0;
    K11.matrix       = Matrix66::Zero();
    K11.matrix(0, 0) = Kx;
    K11.matrix(1, 1) = Ky;

    // k22 --------------
    K22.edge_in  = 1;
    K22.edge_out = 1;
    K22.matrix   = K11.matrix;

    // k12 --------------
    K12.edge_in      = 0;
    K12.edge_out     = 1;
    K12.matrix       = Matrix66::Zero();
    K12.matrix(0, 0) = -Kx;
    K12.matrix(1, 1) = -Ky;
}

void CElementSpringXY::getLocalDoFs(std::vector<used_DoFs_t>& dofs) const
{
    dofs.resize(getNumberEdges());

    used_DoFs_t sDofs = {Kx != 0, Ky != 0, false, false, false, false};
    dofs[0] = dofs[1] = sDofs;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CElementSpringXY::loadParamsFromSet(
    const mrpt::containers::yaml& p, const EvaluationContext& ctx)
{
    if (p.has("Kx")) this->Kx = ctx.evaluate(p["Kx"]);
    if (p.has("Ky")) this->Ky = ctx.evaluate(p["Ky"]);
}

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CElementSpringXY::drawSVG(
    void* _cairo_context, const DrawStructureOptions& options,
    const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    OB_TODO("Implement SVG")
#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CElementSpringXY::do_mesh(
    const size_t my_idx, CStructureProblem& out_fem, MeshOutputInfo& out_info,
    const MeshParams& params)
{
    ASSERT_(conected_nodes_ids.size() == 2);
    ASSERT_(m_parent != nullptr);

    if (out_info.element2nodes.size() <= my_idx)
        out_info.element2nodes.resize(my_idx + 1);

    std::vector<size_t>& my_nodes = out_info.element2nodes.at(my_idx);
    ASSERT_(my_nodes.empty());

    my_nodes.push_back(conected_nodes_ids[0]);
    my_nodes.push_back(conected_nodes_ids[1]);

    if (out_info.element2elements.size() <= my_idx)
        out_info.element2elements.resize(my_idx + 1);

    std::vector<size_t>& my_elements = out_info.element2elements[my_idx];
    ASSERT_(my_elements.empty());

    // Mesh to myself:
    auto new_el = shared_from_this();

    size_t new_el_idx = out_fem.insertElement(new_el);
    my_elements.push_back(new_el_idx);
}
