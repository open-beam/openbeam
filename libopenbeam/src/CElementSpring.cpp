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

#include <openbeam/CElementSpring.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>
#include <openbeam/DrawStructureOptions.h>

#include "internals.h"

using namespace std;
using namespace openbeam;

CElementSpring::CElementSpring() : CElement(2), K(UNINITIALIZED_VALUE) {}

CElementSpring::CElementSpring(
    const size_t from_node_id, const size_t to_node_id, const num_t K)
    : CElement(2, from_node_id, to_node_id), K(K)
{
}

/** Return the stiffness submatrices between each pair of edges in this element,
 * for the current element state.
 */
void CElementSpring::getLocalStiffnessMatrices(
    std::vector<TStiffnessSubmatrix>& outSubMats) const
{
    outSubMats.resize(3);
    TStiffnessSubmatrix& K11 = outSubMats[0];
    TStiffnessSubmatrix& K22 = outSubMats[1];
    TStiffnessSubmatrix& K12 = outSubMats[2];

    if (K == UNINITIALIZED_VALUE)
        throw std::runtime_error(format(
            "CElementSpring: Uninitialized parameter K (Element in %u->%u)",
            static_cast<unsigned int>(this->conected_nodes_ids[0]),
            static_cast<unsigned int>(this->conected_nodes_ids[1])));

    // k11 --------------
    K11.edge_in      = 0;
    K11.edge_out     = 0;
    K11.matrix       = Matrix66::Zero();
    K11.matrix(0, 0) = K;

    // k22 --------------
    K22.edge_in  = 1;
    K22.edge_out = 1;
    K22.matrix   = K11.matrix;

    // k12 --------------
    K12.edge_in      = 0;
    K12.edge_out     = 1;
    K12.matrix       = Matrix66::Zero();
    K12.matrix(0, 0) = -K;
}

void CElementSpring::getLocalDoFs(std::vector<used_DoFs_t>& dofs) const
{
    dofs.resize(getNumberEdges());

    static const used_DoFs_t sDofs = {true, false, false, false, false, false};
    dofs[0] = dofs[1] = sDofs;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CElementSpring::loadParamsFromSet(
    const mrpt::containers::yaml& p, const EvaluationContext& ctx)
{
    this->K = ctx.evaluate(p["K"]);
}

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CElementSpring::drawSVG(
    void* _cairo_context, const DrawStructureOptions& options,
    const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    Cairo::RefPtr<Cairo::Context>& cr =
        *reinterpret_cast<Cairo::RefPtr<Cairo::Context>*>(_cairo_context);

    const double EDGE_WIDTH = 1e-2;
    const double LAT        = 0.08;  // down-scaled, lateral size of spring

    // Draw beam between: node_ids  0 ==> 1
    ASSERT_(conected_nodes_ids.size() == 2);

    // Get original or deformed positions:
    TRotationTrans3D p0 = m_parent->getNodePose(conected_nodes_ids[0]);
    TRotationTrans3D p1 = m_parent->getNodePose(conected_nodes_ids[1]);
    if (!draw_el_params.draw_original_position)
    {  // Deformed position:
        Vector3 pt0_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[0], pt0_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        Vector3 pt1_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[1], pt1_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        for (int l = 0; l < 3; l++)
        {
            p0.t[l] = pt0_deformed[l];
            p1.t[l] = pt1_deformed[l];
        }
    }

    // Unit director vector:
    //    const TPoint3D dir        = (p1.t - p0.t).unitarize();
    const auto p1_p0_norm = (p1.t - p0.t).norm();

    // Draw the spring:
    static const double PTS[] = {
        0,   0,    0, .2,  0,   0, .25, LAT,  0, .35, -LAT, 0, .45, LAT, 0,
        .55, -LAT, 0, .65, LAT, 0, .75, -LAT, 0, .8,  0,    0, 1,   0,   0};

    const size_t          N = sizeof(PTS) / (3 * sizeof(PTS[0]));
    std::vector<TPoint3D> local_pts(N);
    for (size_t i = 0; i < N; i++)
    {
        local_pts[i].x = PTS[3 * i + 0];
        local_pts[i].y = PTS[3 * i + 1];
        local_pts[i].z = PTS[3 * i + 2];
    }

    // Equivalent pose of this element: orientation p0 -> p1
    TRotationTrans3D obj_pose;
    obj_pose.t = p0.t;
    internal::computeOrientationFromTwoPoints(
        p0.t, p1.t, 0 /* rotation on local X */, obj_pose.r);

    // Draw:
    cr->set_source_rgba(0.4, 0.4, 0.4, draw_el_params.color_alpha);
    cr->set_line_width(EDGE_WIDTH);
    internal::drawLocalScaledSegments(
        cr, obj_pose, p1_p0_norm /* scale */, local_pts);

    if (options.show_element_labels)
    {
        cr->set_source_rgb(0, 0, 0.9);

        const double x0 = 0.5 * (p0.t.x + p1.t.x) + 0.5 * options.NODE_RADIUS;
        const double y0 = 0.5 * (p0.t.y + p1.t.y) + 0.5 * options.NODE_RADIUS;
        cr->move_to(x0, y0);
        cr->show_text(openbeam::format(
            "E%u", static_cast<unsigned int>(draw_el_params.element_index)));
    }

#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CElementSpring::do_mesh(
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
