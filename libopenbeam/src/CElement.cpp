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

#include <openbeam/CElement.h>
#include <openbeam/CStructureProblem.h>
#include <openbeam/elements.h>

#include "internals.h"

using namespace std;
using namespace openbeam;

CElement::CElement(unsigned char nEdges)
    : conected_nodes_ids(nEdges),
      m_nEdges(nEdges),
      m_parent(nullptr),
      m_design_rotation_around_linear_axis(0)
{
}

CElement::CElement(
    unsigned char nEdges, const size_t from_node_id, const size_t to_node_id)
    : conected_nodes_ids(nEdges),
      m_nEdges(nEdges),
      m_parent(nullptr),
      m_design_rotation_around_linear_axis(0)
{
    ASSERT_(nEdges >= 2);
    conected_nodes_ids[0] = from_node_id;
    conected_nodes_ids[1] = to_node_id;
}

void CElement::getGlobalStiffnessMatrices(
    std::vector<TStiffnessSubmatrix>& outSubMats) const
{
    // Get local matrices:
    // ======================================================
    this->getLocalStiffnessMatrices(outSubMats);

    // Rotate them
    // ======================================================
    const Matrix33& R =
        m_global_orientation
            .getRot();  // Reference to the 3D rotation matrix (3x3)

    for (size_t i = 0; i < outSubMats.size(); i++)
    {
        Matrix66& M = outSubMats[i].matrix;

        // This is as simple as:
        if ((M.block(0, 0, 3, 3).array() != num_t(0)).any())
            M.block(0, 0, 3, 3) = R * M.block(0, 0, 3, 3) * R.transpose();
        if ((M.block(0, 3, 3, 3).array() != num_t(0)).any())
            M.block(0, 3, 3, 3) = R * M.block(0, 3, 3, 3) * R.transpose();
        if ((M.block(3, 0, 3, 3).array() != num_t(0)).any())
            M.block(3, 0, 3, 3) = R * M.block(3, 0, 3, 3) * R.transpose();
        if ((M.block(3, 3, 3, 3).array() != num_t(0)).any())
            M.block(3, 3, 3, 3) = R * M.block(3, 3, 3, 3) * R.transpose();
    }
}

TRotation3D::TRotation3D(const num_t roll, const num_t pitch, const num_t yaw)
{
    if (roll == 0 && pitch == 0 && yaw == 0)
    {
        m_is_pure_identity = true;
        rot.setIdentity();
    }
    else
    {
        m_is_pure_identity = false;

        const num_t cy = cos(yaw);
        const num_t sy = sin(yaw);
        const num_t cp = cos(pitch);
        const num_t sp = sin(pitch);
        const num_t cr = cos(roll);
        const num_t sr = sin(roll);

        rot(0, 0) = cy * cp;
        rot(0, 1) = cy * sp * sr - sy * cr;
        rot(0, 2) = cy * sp * cr + sy * sr;
        rot(1, 0) = sy * cp;
        rot(1, 1) = sy * sp * sr + cy * cr;
        rot(1, 2) = sy * sp * cr - cy * sr;
        rot(2, 0) = -sp;
        rot(2, 1) = cp * sr;
        rot(2, 2) = cp * cr;
    }
}

void TRotation3D::matrix2angles(
    const Matrix33& R, num_t& roll, num_t& pitch, num_t& yaw)
{
    ASSERTDEB_(
        std::abs(
            sqrt(square(R(0, 0)) + square(R(1, 0)) + square(R(2, 0))) - 1) <
        3e-3)
    ASSERTDEB_(
        std::abs(
            sqrt(square(R(0, 1)) + square(R(1, 1)) + square(R(2, 1))) - 1) <
        3e-3)
    ASSERTDEB_(
        std::abs(
            sqrt(square(R(0, 2)) + square(R(1, 2)) + square(R(2, 2))) - 1) <
        3e-3)

    // Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
    pitch = atan2(-R(2, 0), hypot(R(0, 0), R(1, 0)));  // asin( - R(2,0) );

    // Roll:
    if ((fabs(R(2, 1)) + fabs(R(2, 2))) <
        10 * std::numeric_limits<double>::epsilon())
    {
        // Gimbal lock between yaw and roll. This one is arbitrarily forced to
        // be zero. Check
        // http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html.
        // If cos(pitch)==0, the homogeneous matrix is: When sin(pitch)==1:
        //  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
        //  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
        //  |-1     0         0     z|   |-1    0         0     z|
        //  \0      0         0     1/   \0     0         0     1/
        //
        // And when sin(pitch)=-1:
        //  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
        //  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
        //  |1      0          0     z|   |1    0          0     z|
        //  \0      0          0     1/   \0    0          0     1/
        //
        // Both cases are in a "gimbal lock" status. This happens because pitch
        // is vertical.

        roll = 0.0;
        if (pitch > 0)
            yaw = atan2(R(1, 2), R(0, 2));
        else
            yaw = atan2(-R(1, 2), -R(0, 2));
    }
    else
    {
        roll = atan2(R(2, 1), R(2, 2));
        // Yaw:
        yaw = atan2(R(1, 0), R(0, 0));
    }
}

//  Test for whether this 3x3 matrix is EXACTLY the identity
bool TRotation3D::isIdentity() const
{
    if (m_is_pure_identity)
        return true;  // Avoid checking real numbers (more costly...)
    else
    {
        return rot.coeff(0, 0) == 1 && rot.coeff(1, 1) == 1 &&
               rot.coeff(2, 2) == 1;
    }
}

void CElement::updateOrientationFromNodePositions()
{
    ASSERT_(m_parent);
    ASSERT_(conected_nodes_ids.size() == 2);

    const TRotationTrans3D& p1 = m_parent->getNodePose(conected_nodes_ids[0]);
    const TRotationTrans3D& p2 = m_parent->getNodePose(conected_nodes_ids[1]);

    internal::computeOrientationFromTwoPoints(
        p1.t, p2.t, m_design_rotation_around_linear_axis, m_global_orientation);
}

/** Return the local DoFs transformed with the current element pose in \a
 * m_global_orientation */
void CElement::getGlobalDoFs(std::vector<used_DoFs_t>& dofs) const
{
    // 1: Get local DOFS:
    std::vector<used_DoFs_t> local_dofs;
    getLocalDoFs(local_dofs);

    // 2: Take into account rotations:
    const size_t N = local_dofs.size();
    dofs.resize(N);

    const Matrix33& R = m_global_orientation.getRot();  // Shortcut

    // Binarize R (this is to prevent numeric innacuracies to make us consider
    // more DoFs than the actual ones):
    bool RnonZero[3][3];
    for (size_t r = 0; r < 3; r++)
        for (size_t c = 0; c < 3; c++)
            RnonZero[r][c] = std::abs(R(r, c)) > 1e-10;

    for (size_t i = 0; i < N; i++)
    {
        // If all 3 XYZ are used, don't even lose more time on this...
        if (local_dofs[i][0] && local_dofs[i][1] && local_dofs[i][2])
        { dofs[i][0] = dofs[i][1] = dofs[i][2] = true; }
        else
        {
            // X Y Z
            dofs[i][0] = (local_dofs[i][0] && RnonZero[0][0]) ||
                         (local_dofs[i][1] && RnonZero[0][1]) ||
                         (local_dofs[i][2] && RnonZero[0][2]);
            dofs[i][1] = (local_dofs[i][0] && RnonZero[1][0]) ||
                         (local_dofs[i][1] && RnonZero[1][1]) ||
                         (local_dofs[i][2] && RnonZero[1][2]);
            dofs[i][2] = (local_dofs[i][0] && RnonZero[2][0]) ||
                         (local_dofs[i][1] && RnonZero[2][1]) ||
                         (local_dofs[i][2] && RnonZero[2][2]);
        }

        // If all 3 rotation are used, don't even lose more time on this...
        if (local_dofs[i][3] && local_dofs[i][4] && local_dofs[i][5])
        { dofs[i][3] = dofs[i][4] = dofs[i][5] = true; }
        else
        {
            // thX thY thZ
            dofs[i][3] = (local_dofs[i][3] && RnonZero[0][0]) ||
                         (local_dofs[i][4] && RnonZero[0][1]) ||
                         (local_dofs[i][5] && RnonZero[0][2]);
            dofs[i][4] = (local_dofs[i][3] && RnonZero[1][0]) ||
                         (local_dofs[i][4] && RnonZero[1][1]) ||
                         (local_dofs[i][5] && RnonZero[1][2]);
            dofs[i][5] = (local_dofs[i][3] && RnonZero[2][0]) ||
                         (local_dofs[i][4] && RnonZero[2][1]) ||
                         (local_dofs[i][5] && RnonZero[2][2]);
        }
    }
}

/** Class factory from element name.
 */
CElement::Ptr CElement::createElementByName(const std::string& s)
{
    if (strCmpI("BEAM2D_AA", s)) return std::make_shared<CElementBeam_2D_AA>();
    if (strCmpI("BEAM2D_AR", s)) return std::make_shared<CElementBeam_2D_AR>();
    if (strCmpI("BEAM2D_RA", s)) return std::make_shared<CElementBeam_2D_RA>();
    if (strCmpI("BEAM2D_RR", s)) return std::make_shared<CElementBeam_2D_RR>();
    if (strCmpI("BEAM2D_RD", s)) return std::make_shared<CElementBeam_2D_RD>();
    if (strCmpI("SPRING_1D", s)) return std::make_shared<CElementSpring>();
    if (strCmpI("SPRING_XYZ", s)) return std::make_shared<CElementSpringXYZ>();
    if (strCmpI("SPRING_TORSION", s))
        return std::make_shared<CElementTorsionSpring>();

    return {};
}
