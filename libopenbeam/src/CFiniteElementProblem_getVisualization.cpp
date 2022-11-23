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

#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CText.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>
#include <openbeam/DrawStructureOptions.h>

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;

mrpt::opengl::CSetOfObjects::Ptr CFiniteElementProblem::getVisualization(
    const DrawStructureOptions& o, const StaticSolveProblemInfo& solverInfo,
    const MeshOutputInfo* meshingInfo, const StressInfo* stressInfo) const
{
    auto gl = mrpt::opengl::CSetOfObjects::Create();

    const size_t nNodes = getNumberOfNodes();
    const size_t nEle   = getNumberOfElements();

    const size_t nNodesToDraw =
        meshingInfo ? meshingInfo->num_original_nodes : nNodes;

    num_t min_x, max_x, min_y, max_y;
    getBoundingBox(min_x, max_x, min_y, max_y);

    // "Screen constant sized" objects -------------
    // const double NODE_LABEL_SIZE = o.labels_size;
    // --------------

    // Edges original:  ==============================================
    if (o.show_elements_original)
    {
        DrawStructureOptions opts2 = o;
        opts2.show_element_labels  = false;
        opts2.show_node_labels     = false;
        for (size_t i = 0; i < nEle; i++)
        {
            const auto el = this->getElement(i);
            ASSERT_(el != nullptr);

            DrawElementExtraParams el_params;
            el_params.element_index          = i;
            el_params.color_alpha            = o.elements_original_alpha;
            el_params.draw_original_position = true;

            gl->insert(el->getVisualization(opts2, el_params, meshingInfo));
        }
    }

    // Nodes original:  ==============================================
    if (o.show_nodes_original)
    {
        for (size_t i = 0; i < nNodesToDraw; i++)
        {
            const TRotationTrans3D& p = this->getNodePose(i);

            {
                auto glNode = mrpt::opengl::CSphere::Create();
                glNode->setRadius(o.NODE_RADIUS);
                glNode->setColor(0, 0, 0, o.nodes_original_alpha);
                glNode->setNumberDivsLatitude(o.NODE_SPHERE_DIVS);
                glNode->setNumberDivsLongitude(o.NODE_SPHERE_DIVS);
                glNode->setLocation(p.t);
                gl->insert(glNode);
            }

            if (o.show_node_labels && !o.show_nodes_deformed)
            {
                auto glLb = mrpt::opengl::CText::Create();
                glLb->setColor_u8(0x00, 0x00, 0xd0);
                glLb->setLocation(p.t + TPoint3D(1.3, 0.8, 0) * o.NODE_RADIUS);
                glLb->setString(getNodeLabel(i));
                gl->insert(glLb);
            }
        }
    }

    // If there is anything to draw in "deformed", determine the scale:
    // =======================================================================
    num_t DEFORMED_SCALE_FACTOR = o.deformed_scale_factor;
    if (DEFORMED_SCALE_FACTOR == 0 &&
        (o.show_nodes_deformed || o.show_elements_deformed))
    {  // Autoscale of deformations:
        const num_t max_desired_deformation =
            o.deformed_scale_auto_max_image_ratio *
            std::max(max_x - min_x, max_y - min_y);
        const num_t max_real_deformation =
            this->getMaximumDeformedDisplacement(solverInfo);
        DEFORMED_SCALE_FACTOR =
            max_real_deformation == 0
                ? 1
                : max_desired_deformation / max_real_deformation;
    }

    // Edges deformed:  ==============================================
    if (o.show_elements_deformed)
    {
        for (size_t i = 0; i < nEle; i++)
        {
            const auto el = this->getElement(i);
            ASSERT_(el != nullptr);

            DrawElementExtraParams el_params;
            el_params.element_index          = i;
            el_params.color_alpha            = o.elements_deformed_alpha;
            el_params.draw_original_position = false;  // Draw deformed
            el_params.solver_info            = &solverInfo;
            el_params.deformed_scale_factor  = DEFORMED_SCALE_FACTOR;

            gl->insert(el->getVisualization(o, el_params, meshingInfo));
        }
    }

    // Nodes deformed:  ==============================================
    if (o.show_nodes_deformed)
    {
        for (size_t i = 0; i < nNodesToDraw; i++)
        {
            Vector3 pt;
            this->getNodeDeformedPosition(
                i, pt, solverInfo, DEFORMED_SCALE_FACTOR);

            {
                auto glNode = mrpt::opengl::CSphere::Create();
                glNode->setRadius(o.NODE_RADIUS);
                glNode->setColor(0, 0, 0, o.nodes_deformed_alpha);
                glNode->setNumberDivsLatitude(o.NODE_SPHERE_DIVS);
                glNode->setNumberDivsLongitude(o.NODE_SPHERE_DIVS);
                glNode->setLocation(pt.x(), pt.y(), pt.z());
                gl->insert(glNode);
            }

            if (o.show_node_labels)
            {
                auto glLb = mrpt::opengl::CText::Create();
                glLb->setColor_u8(0x00, 0x00, 0xd0);
                glLb->setLocation(
                    TPoint3D(pt.x(), pt.y(), pt.z()) +
                    TPoint3D(1.3, 0.8, 0) * o.NODE_RADIUS);
                glLb->setString(getNodeLabel(i));
                gl->insert(glLb);
            }
        }
    }

    // Bounding conditions: ==============================================
    if (o.show_constraints)
    {
        internal_getVisualization_constraints(
            *gl, o, solverInfo, meshingInfo, DEFORMED_SCALE_FACTOR);
    }

    // Draw concentrated loads =========================================
    if (o.show_loads)
    {
        internal_getVisualization_nodeLoads(
            *gl, o, solverInfo, meshingInfo, DEFORMED_SCALE_FACTOR);
    }

    // Draw distributed loads on elements ==============================
    auto str = dynamic_cast<const CStructureProblem*>(this);
    if (str && o.show_loads)
    {
        internal_getVisualization_distributedLoads(
            *str, *gl, o, solverInfo, meshingInfo, DEFORMED_SCALE_FACTOR);
    }

    // Draw stress diagrams on elements ==============================
    if (stressInfo && o.show_any_stress())
    {
        internal_getVisualization_stressDiagrams(
            *gl, o, solverInfo, meshingInfo, DEFORMED_SCALE_FACTOR,
            *stressInfo);
    }

    return gl;
}

void CFiniteElementProblem::internal_getVisualization_nodeLoads(
    mrpt::opengl::CSetOfObjects& gl, const DrawStructureOptions& o,
    const StaticSolveProblemInfo& solver_info,
    const MeshOutputInfo* meshing_info, num_t DEFORMED_SCALE_FACTOR) const
{
    num_t min_x, max_x, min_y, max_y;
    getBoundingBox(min_x, max_x, min_y, max_y);

    // Dimensions of constraint plots:
    const double R = o.NODE_RADIUS;

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
        std::max(max_x - min_x, max_y - min_y) * o.node_loads_max_relative_size;
    const double FORCE_SCALE_FACTOR =
        max_load_force == 0 ? 1 : max_desired_abs_force_len / max_load_force;
    const double TORQUE_SCALE_FACTOR =
        max_load_torque == 0 ? 1 : max_desired_abs_force_len / max_load_torque;

    // 1) Forces on nodes
    // -----------------------------------
    for (const auto& load : m_loads_at_each_dof)
    {
        const size_t dof_idx   = load.first;
        const num_t  force_val = load.second;
        if (force_val == 0) continue;  // May happen sometimes.

        const NodeDoF& dof_info = m_problem_DoFs[dof_idx];

        // Scaled force dimension:
        const double LEN = std::abs(force_val) * (dof_info.dofAsInt() < 3
                                                      ? FORCE_SCALE_FACTOR
                                                      : TORQUE_SCALE_FACTOR);

        // Draw external force on "dof_info.dof" [0,5]:

        // 1st: generate shape of the force in "local coords:"
        //  Assume possitive sign (will be scaled below)
        std::vector<TPoint3D> seq_points_local;
        switch (dof_info.dof)
        {
            case DoF_index::DX:
                seq_points_local.push_back(TPoint3D(-R - LEN, 0, 0));
                seq_points_local.push_back(TPoint3D(-R, 0, 0));
                break;
            case DoF_index::DY:
                seq_points_local.push_back(TPoint3D(0, -R - LEN, 0));
                seq_points_local.push_back(TPoint3D(0, -R, 0));
                break;
            case DoF_index::DZ:
                seq_points_local.push_back(TPoint3D(0, 0, -R - LEN));
                seq_points_local.push_back(TPoint3D(0, 0, -R));
                break;
            case DoF_index::RX:
                OB_TODO("Implement M torques")
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
                seq_points_local[k] = -seq_points_local[k];

        TRotationTrans3D node_pose;

        if (!o.show_nodes_deformed)
        {  // original:
            node_pose = this->getNodePose(dof_info.nodeId);
        }
        else
        {  // deformed:
            node_pose = this->getNodePose(dof_info.nodeId);
            Vector3 pt;
            this->getNodeDeformedPosition(
                dof_info.nodeId, pt, solver_info, DEFORMED_SCALE_FACTOR);
            for (int k = 0; k < 3; k++) node_pose.t[k] = pt[k];
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
                seq_points_global[k] += node_pose.t;
        }
        else
        {
            seq_points_global.resize(nPts);
            for (size_t k = 0; k < nPts; k++)
            {
                // POINT_GLOBAL = NODE_GLOBAL + MATRIX33 * POINT_LOCAL
                for (int l = 0; l < 3; l++)
                    seq_points_global[k][l] =
                        node_pose.t[l] +
                        node_rot.coeff(l, 0) * seq_points_local[k][0] +
                        node_rot.coeff(l, 1) * seq_points_local[k][1] +
                        node_rot.coeff(l, 2) * seq_points_local[k][2];
            }
        }

        auto glNodeLoad = mrpt::opengl::CArrow::Create();
        glNodeLoad->setColor(
            0, 0, 1,
            o.show_nodes_deformed ? o.loads_deformed_alpha
                                  : o.loads_original_alpha);
        glNodeLoad->setHeadRatio(0.2f);
        glNodeLoad->setSmallRadius(1.1 * o.BEAM_PINNED_RADIUS);
        glNodeLoad->setLargeRadius(2.0 * o.BEAM_PINNED_RADIUS);
        glNodeLoad->setArrowEnds(
            seq_points_global.at(0), seq_points_global.at(1));
        gl.insert(glNodeLoad);
    }
}

void CFiniteElementProblem::internal_getVisualization_constraints(
    mrpt::opengl::CSetOfObjects& gl, const DrawStructureOptions& o,
    const StaticSolveProblemInfo& solver_info,
    const MeshOutputInfo* meshing_info, num_t DEFORMED_SCALE_FACTOR) const
{
    using PT3 = mrpt::math::TPoint3D;
    using namespace mrpt;

    // Dimensions of constraint plots:
    const double R     = o.BEAM_PINNED_RADIUS;
    const double SCALE = R * 3.0;
    const double W     = SCALE * 1.2;
    const double H     = SCALE * 1.8;
    const double wR    = W / 3;  // Wheels radius

    // Run the whole thing twice:
    // 1) Draw original constraints.
    // 2) Draw deformed constraints (only if "show_nodes_deformed")
    const int num_layers = o.show_nodes_deformed ? 2 : 1;

    using node_idx_t = std::size_t;

    std::map<node_idx_t, std::set<uint8_t>> nodeConstrained;

    for (const auto& kv : m_DoF_constraints)
    {
        // Retrieve the 3D orientation of the node:
        const size_t dof_idx = kv.first;

        OB_TODO("Draw constrained values != 0");
        // const num_t  const_value = kv.second;

        const NodeDoF& dofInfo = m_problem_DoFs[dof_idx];

        nodeConstrained[dofInfo.nodeId].insert(dofInfo.dofAsInt());
    }

    OB_TODO("Rotation according to element orientation");

    for (const auto& kv : nodeConstrained)
    {
        const auto nodeId     = kv.first;
        const auto constrDoFs = kv.second;

        // Build set into a bit field for convenience:
        uint8_t constrBits = 0;
        for (const auto bit : constrDoFs) constrBits = constrBits | (1 << bit);

        // Build list of lines to draw depending on constraint, in local coords:
        std::vector<mrpt::math::TSegment3D> sgms;

        auto lambdaRotateAllSegms = [&sgms](double ang) {
            const auto r = mrpt::poses::CPose3D::FromYawPitchRoll(ang, 0, 0);
            for (auto& sg : sgms)
            {
                sg.point1 = r.rotateVector(sg.point1);
                sg.point2 = r.rotateVector(sg.point2);
            }
        };

        // Add a "wheel" at the given top point (x,y):
        auto lambdaAddWheel = [&sgms](double x, double y, double R) {
            const size_t                      nPts = 10;
            std::vector<mrpt::math::TPoint3D> pts;
            for (size_t i = 0; i < nPts; i++)
            {
                double ang =
                    M_PI * 0.5 + 2 * M_PI * i / static_cast<double>(nPts);
                pts.emplace_back(x + cos(ang) * R, y - R + sin(ang) * R, 0);
            }

            for (size_t i = 0; i < nPts; i++)
                sgms.emplace_back(pts.at(i), pts.at((i + 1) % nPts));
        };

        // Add a "ground line" at the given top point (x,y):
        auto lambdaAddGroundSymbol =
            [&sgms](double x, double y, double L, double rotation = 0) {
                const size_t nLines = 6;
                const double dx     = 1.0 * L / static_cast<double>(nLines - 1);

                const auto rot =
                    mrpt::poses::CPose3D::FromYawPitchRoll(rotation, 0, 0);

                sgms.emplace_back(
                    PT3(x, y, 0) + rot.rotateVector({0, -L * 0.5, 0}),
                    PT3(x, y, 0) + rot.rotateVector({0, +L * 0.5, 0}));

                for (size_t i = 0; i < nLines; i++)
                {
                    double y0 =
                        -L * 0.5 + L * i / static_cast<double>(nLines - 1);

                    sgms.emplace_back(
                        PT3(x, y, 0) + rot.rotateVector({0, y0, 0}),
                        PT3(x, y, 0) + rot.rotateVector({-dx, y0 + dx, 0}));
                }
            };

        // DOFs:
        //      RR RDDD
        //      ZY XZYX
        //  0b0000 0011  = 0x03 // DX DY
        //  0b0000 0001  = 0x01 // DX
        //  0b0000 0010  = 0x02 // DY
        //  0b0010 0011  = 0x23 // DX DY ROTZ
        //  0b0010 0010  = 0x22 // DY ROTZ
        //

        switch (constrBits)
        {
            case 0x03:
                // xy only:
                sgms.emplace_back(PT3(0, -R, 0), PT3(W, -R - H, 0));
                sgms.emplace_back(PT3(W, -R - H, 0), PT3(-W, -R - H, 0));
                sgms.emplace_back(PT3(-W, -R - H, 0), PT3(0, -R, 0));

                lambdaAddGroundSymbol(0, -R - H, 2 * W, 90.0_deg);
                break;

            case 0x01:
            {
                // x only:
                sgms.emplace_back(PT3(-R, 0, 0), PT3(-R - H, W, 0));
                sgms.emplace_back(PT3(-R - H, W, 0), PT3(-R - H, -W, 0));
                sgms.emplace_back(PT3(-R - H, -W, 0), PT3(-R, 0, 0));
                sgms.emplace_back(PT3(-R, 0, 0), PT3(-R - H, W, 0));

                lambdaAddWheel(-R - H - wR, W * 0.8 + wR, wR);
                lambdaAddWheel(-R - H - wR, 0 + wR, wR);
                lambdaAddWheel(-R - H - wR, -W * 0.8 + wR, wR);

                lambdaAddGroundSymbol(-R - H - 2 * W / 3, 0, 2 * W, 0.0_deg);

                double angX = 0, angY = 0, angZ = 0;
                TRotation3D::matrix2angles(
                    m_nodeMainDirection.at(nodeId).getRot(), angX, angY, angZ);

                // flip horizontally?
                if (cos(angZ) < 0) lambdaRotateAllSegms(180.0_deg);
            }
            break;

            case 0x02:
            {
                // y only:
                sgms.emplace_back(PT3(0, -R, 0), PT3(W, -R - H, 0));
                sgms.emplace_back(PT3(W, -R - H, 0), PT3(-W, -R - H, 0));
                sgms.emplace_back(PT3(-W, -R - H, 0), PT3(0, -R, 0));
                sgms.emplace_back(PT3(0, -R, 0), PT3(W, -R - H, 0));

                lambdaAddWheel(W * 0.8, -R - H, wR);
                lambdaAddWheel(0, -R - H, wR);
                lambdaAddWheel(-W * 0.8, -R - H, wR);

                lambdaAddGroundSymbol(0, -R - H - 2 * W / 3, 2 * W, 90.0_deg);

                double angX = 0, angY = 0, angZ = 0;
                TRotation3D::matrix2angles(
                    m_nodeMainDirection.at(nodeId).getRot(), angX, angY, angZ);

                // flip vertically?
                if (sin(angZ) < 0) lambdaRotateAllSegms(180.0_deg);
            }
            break;

            case 0x23:
            {
                // 2D fix:
                sgms.emplace_back(PT3(-R, H, 0), PT3(-R, -H, 0));

                double angX = 0, angY = 0, angZ = 0;
                TRotation3D::matrix2angles(
                    m_nodeMainDirection.at(nodeId).getRot(), angX, angY, angZ);

                lambdaAddGroundSymbol(-R, 0, 2 * W);

                lambdaRotateAllSegms(angZ);
            }
            break;

            case 0x22:
            {
                // Y and ROTZ:
                sgms.emplace_back(PT3(-H, -R, 0), PT3(-H, -2 * R, 0));
                sgms.emplace_back(PT3(-H, -2 * R, 0), PT3(H, -2 * R, 0));
                sgms.emplace_back(PT3(H, -2 * R, 0), PT3(H, -R, 0));
                sgms.emplace_back(PT3(H, -R, 0), PT3(-H, -R, 0));

                lambdaAddWheel(W * 0.8, -2 * R, wR);
                lambdaAddWheel(0, -2 * R, wR);
                lambdaAddWheel(-W * 0.8, -2 * R, wR);

                lambdaAddGroundSymbol(0, -2 * R - 2 * wR, 2 * H, 90.0_deg);
            }
            break;

            default:
                throw std::runtime_error(mrpt::format(
                    "Not implemented constrained bit field: 0x%02X",
                    constrBits));
        };

        // Draw:
        for (int pass = 0; pass < num_layers; pass++)
        {
            TRotationTrans3D node_pose;

            auto glConstr = mrpt::opengl::CSetOfLines::Create();

            if (pass == 0)
            {  // original:

                glConstr->setColor(.0f, .0f, .0f, o.constraints_original_alpha);

                node_pose = this->getNodePose(nodeId);
            }
            else if (pass == 1)
            {  // deformed:
                glConstr->setColor(.0f, .0f, .0f, o.constraints_deformed_alpha);

                node_pose = this->getNodePose(nodeId);

                Vector3 pt;
                this->getNodeDeformedPosition(
                    nodeId, pt, solver_info, DEFORMED_SCALE_FACTOR);

                for (int k = 0; k < 3; k++) node_pose.t[k] = pt[k];
            }

            // Draw:

            ASSERT_(sgms.size() >= 2);
            glConstr->appendLines(sgms);

            OB_TODO("Check nodal coordinates orientation?");
            glConstr->setPose(mrpt::poses::CPose3D::FromRotationAndTranslation(
                node_pose.r.getRot(), node_pose.t));

            gl.insert(glConstr);
        }  // end for "pass" (original/deformed)
    }
}

void CFiniteElementProblem::internal_getVisualization_distributedLoads(
    const CStructureProblem& str, mrpt::opengl::CSetOfObjects& gl,
    const DrawStructureOptions& o, const StaticSolveProblemInfo& solver_info,
    const MeshOutputInfo* meshing, num_t DEFORMED_SCALE_FACTOR) const
{
    DrawElementExtraParams elParams;
    elParams.color_alpha =
        o.show_nodes_deformed ? o.loads_deformed_alpha : o.loads_original_alpha;
    elParams.draw_original_position = !o.show_nodes_deformed;
    elParams.solver_info            = &solver_info;
    elParams.deformed_scale_factor  = DEFORMED_SCALE_FACTOR;

    for (int stage = 0; stage < 3; stage++)
    {
        for (const auto& eLoadKV : str.loadsOnBeams())
        {
            const auto [beamId, eLoad] = eLoadKV;
            if (!eLoad) continue;
            elParams.element_index = beamId;
            switch (stage)
            {
                case 0:
                    eLoad->getVisualization_init(*this, o, elParams, meshing);
                    break;
                case 1:
                    eLoad->getVisualization_pre(*this, o, elParams, meshing);
                    break;
                case 2:
                    gl.insert(
                        eLoad->getVisualization(*this, o, elParams, meshing));
                    break;
            }
        }
    }
}

void CFiniteElementProblem::internal_getVisualization_stressDiagrams(
    mrpt::opengl::CSetOfObjects& gl, const DrawStructureOptions& options,
    const StaticSolveProblemInfo& solverInfo, const MeshOutputInfo* meshingInfo,
    num_t DEFORMED_SCALE_FACTOR, const StressInfo& stressInfo) const
{
    ASSERTMSG_(
        meshingInfo,
        "Doing meshing is required at present for drawing stress diagrams.");

    // The max. absolute value value for each stress:
    FaceStress maxAbsStress;

    std::array<bool, 6> diagEnabled = {
        options.show_force_axial,      options.show_force_shear_y,
        options.show_bending_moment_z, options.show_force_shear_z,
        options.show_bending_moment_y, options.show_torsion_moment};

    for (int pass = 0; pass < 2; pass)
    {
        // pass #0: find out absolute maximums
        // pass #1: Draw them

        for (element_index_t elIdx = 0;
             elIdx < meshingInfo->element2elements.size(); elIdx++)
        {
            const auto& elEls   = meshingInfo->element2elements[elIdx];
            const auto& elNodes = meshingInfo->element2nodes[elIdx];

            mrpt::opengl::CSetOfLines::Ptr glDiag[6];
            if (pass == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (diagEnabled[i])
                    {
                        glDiag[i] = mrpt::opengl::CSetOfLines::Create();
                        glDiag[i]->setColor_u8(mrpt::img::TColor::blue());
                    }
                }
            }

            // Note the "<=" below: we draw the last element twice, once to draw
            // its first face, another for the second face:
            for (size_t iSubEl = 0; iSubEl <= elEls.size(); iSubEl++)
            {
                const auto subElIdx =
                    elEls.at(std::min(iSubEl, elEls.size() - 1));
                const auto& stress = stressInfo.element_stress.at(subElIdx);

                // We only have linear elements yet!
                ASSERT_(stress.size() == 2);

                // Draw the "left" (first) face for all
                unsigned int face = iSubEl + 1 < elEls.size() ? 0 : 1;

                const FaceStress& es =
                    stressInfo.element_stress[subElIdx][face];

                switch (pass)
                {
                    case 0:
                        for (int i = 0; i < 6; i++)
                            mrpt::keep_max(maxAbsStress[i], std::abs(es[i]));
                        break;

                    case 1:  // Draw them:
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            if (auto glLine = glDiag[i]; glLine)
                            {
                                // get coords of the two ends of the
                                // sub-element:
                                const auto n0 =
                                    getElement(subElIdx)->conected_nodes_ids.at(
                                        0);
                                const auto n1 =
                                    getElement(subElIdx)->conected_nodes_ids.at(
                                        1);
                                const auto p0 = getNodePose(n0);
                                const auto p1 = getNodePose(n1);
                                const auto u  = (p1.t - p0.t).unitarize();
                                const mrpt::math::TPoint2D uv = {u.y, -u.x};

                                const double s =
                                    1 * es[i] /
                                    (maxAbsStress[i] != 0 ? maxAbsStress[i]
                                                          : 1.0);

                                const auto pp0 = p0.t + uv * s;
                                const auto pp1 = p1.t + uv * s;
                                glLine->appendLine(
                                    pp0.x, pp0.y, 0, pp1.x, pp1.y, 0);
                            }
                        }
                    }
                    break;
                };

            }  // for each iSubEl

            if (pass == 1)
            {
                for (int i = 0; i < 6; i++)
                    if (diagEnabled[i]) gl.insert(glDiag[i]);
            }

        }  // for each elIdx
    }  // for each pass
}
