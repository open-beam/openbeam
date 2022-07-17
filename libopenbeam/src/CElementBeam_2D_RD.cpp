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

#include <openbeam/CElementBeam_2D_RD.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>

using namespace std;
using namespace openbeam;

CElementBeam_2D_RD::CElementBeam_2D_RD() : CBaseElementBeam(false, true) {}

CElementBeam_2D_RD::CElementBeam_2D_RD(
    const size_t from_node_id, const size_t to_node_id)
    : CBaseElementBeam(from_node_id, to_node_id, false, true)
{
}

/** Return the stiffness submatrices between each pair of edges in this element,
 * for the current element state.
 */
void CElementBeam_2D_RD::getLocalStiffnessMatrices(
    std::vector<TStiffnessSubmatrix>& outSubMats) const
{
    outSubMats.resize(3);
    TStiffnessSubmatrix& K11 = outSubMats[0];
    TStiffnessSubmatrix& K22 = outSubMats[1];
    TStiffnessSubmatrix& K12 = outSubMats[2];

    // Compute my current length by getting the poses of the two nodes at each
    // end of the beam:
    ASSERTDEB_(conected_nodes_ids.size() == 2)
    ASSERTDEB_(m_parent != nullptr)

    const TRotationTrans3D& node0 =
        m_parent->getNodePose(conected_nodes_ids[0]);
    const TRotationTrans3D& node1 =
        m_parent->getNodePose(conected_nodes_ids[1]);

    const num_t L2 = (node1.t - node0.t).sqrNorm();
    const num_t L  = std::sqrt(L2);
    const num_t L3 = L2 * L;

    ASSERT_(L > 0);
    if (E == UNINITIALIZED_VALUE)
        throw std::runtime_error(format(
            "CElementBeam_2D_RD: Uninitialized parameter E (Element in %u->%u)",
            static_cast<unsigned int>(this->conected_nodes_ids[0]),
            static_cast<unsigned int>(this->conected_nodes_ids[1])));
    if (A == UNINITIALIZED_VALUE)
        throw std::runtime_error(format(
            "CElementBeam_2D_RD: Uninitialized parameter A (Element in %u->%u)",
            static_cast<unsigned int>(this->conected_nodes_ids[0]),
            static_cast<unsigned int>(this->conected_nodes_ids[1])));
    if (Iz == UNINITIALIZED_VALUE)
        throw std::runtime_error(format(
            "CElementBeam_2D_RD: Uninitialized parameter Iz (Element in "
            "%u->%u)",
            static_cast<unsigned int>(this->conected_nodes_ids[0]),
            static_cast<unsigned int>(this->conected_nodes_ids[1])));

    // Common part:
    K11.edge_in  = 0;
    K11.edge_out = 0;
    //
    K22.edge_in  = 1;
    K22.edge_out = 1;
    //
    K12.edge_in  = 0;
    K12.edge_out = 1;

    /*
    Kr =
    [  (A*E)/L, 0,        0, |  -(A*E)/L,        0]
    [        0, 0,        0, |         0,        0]
    [        0, 0,  (E*I)/L, |         0, -(E*I)/L]
    --------------------------------------------
    [ -(A*E)/L, 0,        0, |   (A*E)/L,        0]
    [        0, 0, -(E*I)/L, |         0,  (E*I)/L]
    */
    // k11 --------------
    K11.matrix       = Matrix66::Zero();
    K11.matrix(0, 0) = E * A / L;
    K11.matrix(5, 5) = E * Iz / L;

    // k22 --------------
    K22.matrix       = Matrix66::Zero();
    K22.matrix(0, 0) = K11.matrix(0, 0);
    K22.matrix(5, 5) = K11.matrix(5, 5);

    // k12 --------------
    K12.matrix       = Matrix66::Zero();
    K12.matrix(0, 0) = -K11.matrix(0, 0);  // E*A/L;
    K12.matrix(5, 5) = -K11.matrix(5, 5);
}

void CElementBeam_2D_RD::getLocalDoFs(std::vector<used_DoFs_t>& dofs) const
{
    dofs.resize(size_t(getNumberEdges()));

    static const used_DoFs_t sDofsR = {true, true, false, false, false, true};
    static const used_DoFs_t sDofsD = {true, false, false, false, false, true};

    dofs[0] = sDofsR;
    dofs[1] = sDofsD;
}
