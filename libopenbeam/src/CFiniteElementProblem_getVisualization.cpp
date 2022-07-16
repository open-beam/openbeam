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

#include <openbeam/CFiniteElementProblem.h>

#include "internals.h"

using namespace std;
using namespace openbeam;
using namespace Eigen;

mrpt::opengl::CSetOfObjects::Ptr CFiniteElementProblem::getVisualization(
    const RenderInitData& ri, const DrawStructureOptions& options,
    const StaticSolveProblemInfo& solver_info,
    const MeshOutputInfo*         meshing_info) const
{
    auto gl = mrpt::opengl::CSetOfObjects::Create();

    const size_t nNodes = getNumberOfNodes();
    const size_t nEle   = getNumberOfElements();

    const size_t nNodesToDraw =
        meshing_info ? meshing_info->num_original_nodes : nNodes;

    const num_t min_x = ri.min_x, max_x = ri.max_x;
    const num_t min_y = ri.min_y, max_y = ri.max_y;

    // "Screen constant sized" objects -------------
    const double NODE_LABEL_SIZE = options.labels_size / ri.scaleFactor;
    const_cast<DrawStructureOptions*>(&options)->node_radius =
        3 / ri.scaleFactor;
    // --------------

    // Edges original:  ==============================================
    if (options.show_elements_original)
    {
        DrawStructureOptions opts2 = options;
        opts2.show_element_labels  = false;
        opts2.show_node_labels     = false;
        for (size_t i = 0; i < nEle; i++)
        {
            const auto el = this->getElement(i);
            ASSERT_(el != nullptr);

            DrawElementExtraParams el_params;
            el_params.element_index          = i;
            el_params.color_alpha            = options.elements_original_alpha;
            el_params.draw_original_position = true;

            gl->insert(
                el->getVisualization(opts2, ri, el_params, meshing_info));
        }
    }

    // Nodes original:  ==============================================
    if (options.show_nodes_original)
    {
        // cr->set_source_rgba(0, 0, 0, options.nodes_original_alpha);

        for (size_t i = 0; i < nNodesToDraw; i++)
        {
            const TRotationTrans3D& p = this->getNodePose(i);
#if 0
            cr->move_to(p.t.coords[0], p.t.coords[1]);
            cr->arc(
                p.t.coords[0], p.t.coords[1], options.node_radius, 0, 2 * M_PI);
            cr->fill();

            if (options.show_node_labels && !options.show_nodes_deformed)
            {
                cr->move_to(
                    p.t.coords[0] + 1.3 * options.node_radius,
                    p.t.coords[1] + 0.8 * options.node_radius);
                cr->show_text(getNodeLabel(i));
            }
#endif
        }
    }

    // If there is anything to draw in "deformed", determine the scale:
    // ==============================================
    num_t DEFORMED_SCALE_FACTOR = options.deformed_scale_factor;
    if (DEFORMED_SCALE_FACTOR == 0 &&
        (options.show_nodes_deformed || options.show_elements_deformed))
    {  // Autoscale of deformations:
        const num_t max_desired_deformation =
            options.deformed_scale_auto_max_image_ratio *
            std::max(max_x - min_x, max_y - min_y);
        const num_t max_real_deformation =
            this->getMaximumDeformedDisplacement(solver_info);
        DEFORMED_SCALE_FACTOR =
            max_real_deformation == 0
                ? 1
                : max_desired_deformation / max_real_deformation;
    }

    // Edges deformed:  ==============================================
    if (options.show_elements_deformed)
    {
        for (size_t i = 0; i < nEle; i++)
        {
            const auto el = this->getElement(i);
            ASSERT_(el != nullptr);

            DrawElementExtraParams el_params;
            el_params.element_index          = i;
            el_params.color_alpha            = options.elements_deformed_alpha;
            el_params.draw_original_position = false;  // Draw deformed
            el_params.solver_info            = &solver_info;
            el_params.deformed_scale_factor  = DEFORMED_SCALE_FACTOR;

            gl->insert(
                el->getVisualization(options, ri, el_params, meshing_info));
        }
    }

    // Nodes deformed:  ==============================================
    if (options.show_nodes_deformed)
    {
        // cr->set_source_rgba(0, 0, 0, options.nodes_deformed_alpha);

        for (size_t i = 0; i < nNodesToDraw; i++)
        {
            Vector3 pt;
            this->getNodeDeformedPosition(
                i, pt, solver_info, DEFORMED_SCALE_FACTOR);

#if 0
            cr->move_to(pt[0], pt[1]);
            cr->arc(pt[0], pt[1], options.node_radius, 0, 2 * M_PI);
            cr->fill();

            if (options.show_node_labels)
            {
                cr->move_to(
                    pt[0] + 1.3 * options.node_radius,
                    pt[1] + 0.8 * options.node_radius);
                cr->show_text(getNodeLabel(i));
            }
#endif
        }
    }

    // Bounding conditions: ==============================================
    if (options.show_constraints)
    {
        // Dimensions of constraint plots:
        const double R     = options.node_radius;
        const double SCALE = R * 2;  // ...
        const double W     = SCALE * 0.9;
        const double H     = SCALE * 1.8;
        const double H2    = SCALE * 0.8;

        for (const auto& kv : m_DoF_constraints)
        {
            // Retrieve the 3D orientation of the node:
            const size_t dof_idx     = kv.first;
            const num_t  const_value = kv.second;

            const NodeDoF& dof_info = m_problem_DoFs[dof_idx];

            // Draw constraint on "dof_info.dof" [0,5]:

            // 1st: generate shape of constaint in "local coords:"
            std::vector<TPoint3D> seq_points_local;
            switch (dof_info.dof)
            {
                case DoF_index::DX:
                    seq_points_local.push_back(TPoint3D(-R, 0, 0));
                    seq_points_local.push_back(TPoint3D(-R - H, W, 0));
                    seq_points_local.push_back(TPoint3D(-R - H, -W, 0));
                    seq_points_local.push_back(TPoint3D(-R, 0, 0));
                    break;
                case DoF_index::DY:
                    seq_points_local.push_back(TPoint3D(0, -R, 0));
                    seq_points_local.push_back(TPoint3D(W, -R - H, 0));
                    seq_points_local.push_back(TPoint3D(-W, -R - H, 0));
                    seq_points_local.push_back(TPoint3D(0, -R, 0));
                    break;
                case DoF_index::DZ:
                    seq_points_local.push_back(TPoint3D(0, 0, -R));
                    seq_points_local.push_back(TPoint3D(0, W, -R - H));
                    seq_points_local.push_back(TPoint3D(0, -W, -R - H));
                    seq_points_local.push_back(TPoint3D(0, 0, -R));
                    break;
                case DoF_index::RX:
                    seq_points_local.push_back(TPoint3D(-R - H, W, 0));
                    seq_points_local.push_back(TPoint3D(-R - H, -W, 0));
                    seq_points_local.push_back(TPoint3D(-R - H - H2, -W, 0));
                    seq_points_local.push_back(TPoint3D(-R - H - H2, W, 0));
                    seq_points_local.push_back(TPoint3D(-R - H, W, 0));
                    break;
                case DoF_index::RY:
                    seq_points_local.push_back(TPoint3D(W, -R - H, 0));
                    seq_points_local.push_back(TPoint3D(-W, -R - H, 0));
                    seq_points_local.push_back(TPoint3D(-W, -R - H - H2, 0));
                    seq_points_local.push_back(TPoint3D(W, -R - H - H2, 0));
                    seq_points_local.push_back(TPoint3D(W, -R - H, 0));
                    break;
                case DoF_index::RZ:
                    seq_points_local.push_back(TPoint3D(0, W, -R - H));
                    seq_points_local.push_back(TPoint3D(0, -W, -R - H));
                    seq_points_local.push_back(TPoint3D(0, -W, -R - H - H2));
                    seq_points_local.push_back(TPoint3D(0, W, -R - H - H2));
                    seq_points_local.push_back(TPoint3D(0, W, -R - H));
                    break;
            }

            // Run the whole thing twice:
            // 1) Draw original constraints.
            // 2) Draw deformed constraints (only if "show_nodes_deformed")
            const int num_layers = options.show_nodes_deformed ? 2 : 1;

            for (int pass = 0; pass < num_layers; pass++)
            {
                TRotationTrans3D node_pose;

                if (pass == 0)
                {  // original:
#if 0
                    cr->set_line_width(H2 * 0.1);
                    cr->set_source_rgba(
                        0, 0, 0, options.constraints_original_alpha);
#endif
                    node_pose = this->getNodePose(dof_info.nodeId);
                }
                else if (pass == 1)
                {  // deformed:
#if 0
                    cr->set_line_width(H2 * 0.1);
                    cr->set_source_rgba(
                        0, 0, 0, options.constraints_deformed_alpha);
#endif
                    node_pose = this->getNodePose(dof_info.nodeId);
                    Vector3 pt;
                    this->getNodeDeformedPosition(
                        dof_info.nodeId, pt, solver_info,
                        DEFORMED_SCALE_FACTOR);
                    for (int k = 0; k < 3; k++) node_pose.t.coords[k] = pt[k];
                }

                // Draw:
#if 0
                internal::drawLocalScaledSegments(
                    cr, node_pose, 1.0, seq_points_local);
#endif
            }  // end for "pass" (original/deformed)
        }
    }

    // Draw loads ==============================================
    if (options.show_loads)
    {
        // Dimensions of constraint plots:
        const double R                = options.node_radius;
        const double A                = 1.5 * R;  // Arrow size
        const double FORCE_LINE_WIDTH = R * 0.5;

        // 0) Establish scale:
        num_t max_load_force = 0, max_load_torque = 0;
        for (const auto& load : m_loads_at_each_dof)
        {
            const NodeDoF& dof_info  = m_problem_DoFs[load.first];
            const num_t    force_val = std::abs(load.second);

            if (dof_info.dofAsInt() < 3)
                max_load_force = std::max(max_load_force, force_val);
            else
                max_load_torque = std::max(max_load_torque, force_val);
        }
        const double max_desired_abs_force_len =
            std::max(max_x - min_x, max_y - min_y) *
            options.node_loads_max_relative_size;
        const double FORCE_SCALE_FACTOR =
            max_load_force == 0 ? 1
                                : max_desired_abs_force_len / max_load_force;
        const double TORQUE_SCALE_FACTOR =
            max_load_torque == 0 ? 1
                                 : max_desired_abs_force_len / max_load_torque;

        // 1) Forces on nodes
        // -----------------------------------
        for (const auto& load : m_loads_at_each_dof)
        {
            const size_t dof_idx   = load.first;
            const num_t  force_val = load.second;
            if (force_val == 0) continue;  // May happen sometimes.

            const NodeDoF& dof_info = m_problem_DoFs[dof_idx];

            // Scaled force dimension:
            const double LEN = std::abs(force_val) *
                               (dof_info.dofAsInt() < 3 ? FORCE_SCALE_FACTOR
                                                        : TORQUE_SCALE_FACTOR);

            // Draw external force on "dof_info.dof" [0,5]:

            // 1st: generate shape of the force in "local coords:"
            //  Assume possitive sign (will be scaled below)
            std::vector<TPoint3D> seq_points_local;
            switch (dof_info.dof)
            {
                case DoF_index::DX:
                    seq_points_local.push_back(TPoint3D(-R, 0, 0));
                    seq_points_local.push_back(TPoint3D(-R - A, -A, 0));
                    seq_points_local.push_back(TPoint3D(-R, 0, 0));
                    seq_points_local.push_back(TPoint3D(-R - A, A, 0));
                    seq_points_local.push_back(TPoint3D(-R, 0, 0));
                    seq_points_local.push_back(TPoint3D(-R - LEN, 0, 0));
                    break;
                case DoF_index::DY:
                    seq_points_local.push_back(TPoint3D(0, -R, 0));
                    seq_points_local.push_back(TPoint3D(0, -R - LEN, 0));
                    seq_points_local.push_back(TPoint3D(0, -R, 0));
                    seq_points_local.push_back(TPoint3D(-A, -R - A, 0));
                    seq_points_local.push_back(TPoint3D(0, -R, 0));
                    seq_points_local.push_back(TPoint3D(A, -R - A, 0));
                    break;
                case DoF_index::DZ:
                    OB_TODO("Implement Z force")
                    break;
                case DoF_index::RX:
                    break;
                case DoF_index::RY:
                    break;
                case DoF_index::RZ:
                    break;
            }

            // Set correct sign:
            const size_t nPts = seq_points_local.size();
            if (force_val < 0)
                for (size_t k = 0; k < nPts; k++)
                    for (int l = 0; l < 3; l++)
                        seq_points_local[k].coords[l] =
                            -seq_points_local[k].coords[l];

            TRotationTrans3D node_pose;

            if (!options.show_nodes_deformed)
            {  // original:
                //                cr->set_line_width(FORCE_LINE_WIDTH);
                //                cr->set_source_rgba(0, 0, 1,
                //                options.loads_original_alpha);

                node_pose = this->getNodePose(dof_info.nodeId);
            }
            else
            {  // deformed:
                //                cr->set_line_width(FORCE_LINE_WIDTH);
                //                cr->set_source_rgba(0, 0, 1,
                //                options.loads_deformed_alpha);

                node_pose = this->getNodePose(dof_info.nodeId);
                Vector3 pt;
                this->getNodeDeformedPosition(
                    dof_info.nodeId, pt, solver_info, DEFORMED_SCALE_FACTOR);
                for (int k = 0; k < 3; k++) node_pose.t.coords[k] = pt[k];
            }

            const Matrix33& node_rot = node_pose.r.getRot();

            // then rotate to the nodal coordinates (typ. coincides with global
            // coords)
            std::vector<TPoint3D> seq_points_global;
            if (node_pose.r.isIdentity())
            {
                seq_points_global = seq_points_local;
                for (size_t k = 0; k < nPts; k++)
                    // POINT_GLOBAL = NODE_GLOBAL + MATRIX33 * POINT_LOCAL
                    for (int l = 0; l < 3; l++)
                        seq_points_global[k].coords[l] += node_pose.t.coords[l];
            }
            else
            {
                seq_points_global.resize(nPts);
                for (size_t k = 0; k < nPts; k++)
                {
                    // POINT_GLOBAL = NODE_GLOBAL + MATRIX33 * POINT_LOCAL
                    for (int l = 0; l < 3; l++)
                        seq_points_global[k].coords[l] =
                            node_pose.t.coords[l] +
                            node_rot.coeff(l, 0) *
                                seq_points_local[k].coords[0] +
                            node_rot.coeff(l, 1) *
                                seq_points_local[k].coords[1] +
                            node_rot.coeff(l, 2) *
                                seq_points_local[k].coords[2];
                }
            }

            // Draw the lines:
            for (size_t k = 0; k < nPts; k++)
            {
                const double xx = seq_points_global[k].coords[0];
                const double yy = seq_points_global[k].coords[1];
#if 0
                if (k == 0)
                    cr->move_to(xx, yy);
                else
                    cr->line_to(xx, yy);
#endif
            }
#if 0
            cr->set_line_cap(Cairo::LINE_CAP_ROUND);
            cr->set_line_join(Cairo::LINE_JOIN_ROUND);
            cr->stroke();
#endif

        }  // end for each load

    }  // end draw loads

    return gl;
}
