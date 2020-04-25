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

#include <openbeam/CElementSpring.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>
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
    openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t& outSubMats)
    const
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
    K11.matrix       = TMatrix66::Zero();
    K11.matrix(0, 0) = K;

    // k22 --------------
    K22.edge_in  = 1;
    K22.edge_out = 1;
    K22.matrix   = K11.matrix;

    // k12 --------------
    K12.edge_in      = 0;
    K12.edge_out     = 1;
    K12.matrix       = TMatrix66::Zero();
    K12.matrix(0, 0) = -K;
}

void CElementSpring::getLocalDoFs(std::vector<TUsedDoFs>& dofs) const
{
    dofs.resize(getNumberEdges());

    static const TUsedDoFs sDofs = {true, false, false, false, false, false};
    dofs[0] = dofs[1] = sDofs;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CElementSpring::loadParamsFromSet(
    const TParamSet& params, const TEvaluationContext& eval)
{
    for (TParamSet::const_iterator it = params.begin(); it != params.end();
         ++it)
    {
        if (strCmpI(it->first, "K"))
        { eval.parser_evaluate_expression(it->second, this->K); }
    }
}

#if OPENBEAM_HAS_QT5Svg
void CElementSpring::drawQtSVG(
    QSvgGenerator& svg, const TDrawStructureOptions& options,
    const TRenderInitData& ri, const TDrawElementExtraParams& draw_el_params,
    const TMeshOutputInfo* meshing_info) const
{
}
#endif

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CElementSpring::drawSVG(
    void* _cairo_context, const TDrawStructureOptions& options,
    const TRenderInitData& ri, const TDrawElementExtraParams& draw_el_params,
    const TMeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    Cairo::RefPtr<Cairo::Context>& cr =
        *reinterpret_cast<Cairo::RefPtr<Cairo::Context>*>(_cairo_context);

    const double EDGE_WIDTH = 1e-2;
    const double LAT        = 0.08;  // down-scaled, lateral size of spring

    // Draw beam between: node_ids  0 ==> 1
    OBASSERT(conected_nodes_ids.size() == 2)

    // Get original or deformed positions:
    TRotationTrans3D p0 = m_parent->getNodePose(conected_nodes_ids[0]);
    TRotationTrans3D p1 = m_parent->getNodePose(conected_nodes_ids[1]);
    if (!draw_el_params.draw_original_position)
    {  // Deformed position:
        TVector3 pt0_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[0], pt0_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        TVector3 pt1_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[1], pt1_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        for (int l = 0; l < 3; l++)
        {
            p0.t.coords[l] = pt0_deformed[l];
            p1.t.coords[l] = pt1_deformed[l];
        }
    }

    // Unit director vector:
    const TPoint3D p1_p0 = TPoint3D(
        p1.t.coords[0] - p0.t.coords[0], p1.t.coords[1] - p0.t.coords[1],
        p1.t.coords[2] - p0.t.coords[2]);
    const num_t p1_p0_norm = p1_p0.norm();
    TPoint3D    dir;
    if (p1_p0_norm == 0)
        dir = p1_p0;
    else
    {
        const num_t inv = 1 / p1_p0_norm;
        for (int l = 0; l < 3; l++) dir.coords[l] = p1_p0.coords[l] * inv;
    }

    // Draw the spring:
    static const double PTS[] = {
        0,   0,    0, .2,  0,   0, .25, LAT,  0, .35, -LAT, 0, .45, LAT, 0,
        .55, -LAT, 0, .65, LAT, 0, .75, -LAT, 0, .8,  0,    0, 1,   0,   0};

    const size_t          N = sizeof(PTS) / (3 * sizeof(PTS[0]));
    std::vector<TPoint3D> local_pts(N);
    for (size_t i = 0; i < N; i++)
    {
        local_pts[i].coords[0] = PTS[3 * i + 0];
        local_pts[i].coords[1] = PTS[3 * i + 1];
        local_pts[i].coords[2] = PTS[3 * i + 2];
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

        const double x0 =
            0.5 * (p0.t.coords[0] + p1.t.coords[0]) + 0.5 * options.node_radius;
        const double y0 =
            0.5 * (p0.t.coords[1] + p1.t.coords[1]) + 0.5 * options.node_radius;
        cr->move_to(x0, y0);
        cr->show_text(openbeam::format(
            "E%u", static_cast<unsigned int>(draw_el_params.element_index)));
    }

#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CElementSpring::do_mesh(
    const size_t my_idx, CStructureProblem& out_fem, TMeshOutputInfo& out_info,
    const TMeshParams& params)
{
    throw std::runtime_error("TO DO");
}
