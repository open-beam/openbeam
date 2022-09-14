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
#include <openbeam/DrawStructureOptions.h>

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;

CFiniteElementProblem::~CFiniteElementProblem() = default;

void CFiniteElementProblem::clear()
{
    m_node_poses.clear();
    m_DoF_constraints.clear();
    m_loads_at_each_dof.clear();
    m_loads_at_each_dof_equivs.clear();
    m_extra_stress_for_each_element.clear();
    m_elements.clear();
}

size_t CFiniteElementProblem::insertElement(CElement::Ptr el)
{
    ASSERT_(el);
    el->setParent(this);
    const size_t idx = m_elements.size();
    m_elements.push_back(el);
    return idx;
}

/** Return a constant pointer to the i'th element in the structure. */
CElement::ConstPtr CFiniteElementProblem::getElement(size_t i) const
{
    return m_elements.at(i);
}

/** Return a pointer to the i'th element in the structure. */
const CElement::Ptr& CFiniteElementProblem::getElement(size_t i)
{
    return m_elements.at(i);
}

/** Insert a new constraint in the structure (this objects owns the memory, will
 * delete the element object when not needed anymore). \return The constraint
 * index */
void CFiniteElementProblem::insertConstraint(
    const size_t dof_index, const num_t value)
{
    if (m_problem_DoFs.empty())
        throw std::runtime_error(
            "insertConstraint(): Empty DOFs (Have you called updateAll() "
            "first?)");
    ASSERT_(dof_index < m_problem_DoFs.size());
    m_DoF_constraints[dof_index] = value;
}

/** Sets the load (force or moment) applied to a given DOF  \sa addLoadAtDOF */
void CFiniteElementProblem::setLoadAtDOF(const size_t dof_index, const num_t f)
{
    if (m_problem_DoFs.empty())
        throw std::runtime_error(
            "setLoadAtDOF(): Empty DOFs (Have you called updateAll() first?)");
    ASSERT_(dof_index < m_problem_DoFs.size());
    m_loads_at_each_dof[dof_index] = f;
}

/** Adds a load (force or moment) to a given DOF, accumulating to previously
 * existing values if any.  \sa setLoadAtDOF */
void CFiniteElementProblem::addLoadAtDOF(const size_t dof_index, const num_t f)
{
    if (m_problem_DoFs.empty())
        throw std::runtime_error(
            "addLoadAtDOF(): Empty DOFs (Have you called updateAll() first?)");
    ASSERT_(dof_index < m_problem_DoFs.size());
    m_loads_at_each_dof[dof_index] += f;
}

void CFiniteElementProblem::setNumberOfNodes(size_t N)
{
    m_node_defined.resize(N);
    m_node_poses.resize(N);
    m_node_labels.resize(N);
}

void CFiniteElementProblem::setNodePose(size_t idx, const TRotationTrans3D& p)
{
    m_node_poses.at(idx)        = p;
    m_node_defined.at(idx).used = true;
}

void CFiniteElementProblem::setNodePose(
    size_t idx, const num_t x, const num_t y, const num_t z)
{
    TRotationTrans3D& P = m_node_poses.at(idx);
    P.t.x               = x;
    P.t.y               = y;
    P.t.z               = z;

    m_node_defined.at(idx).used = true;
}

size_t CFiniteElementProblem::insertNode(const TRotationTrans3D& p)
{
    const size_t idx = m_node_poses.size();
    m_node_poses.push_back(p);

    if (m_node_defined.size() < idx + 1) m_node_defined.resize(idx + 1);
    if (m_node_labels.size() < idx + 1) m_node_labels.resize(idx + 1);

    m_node_defined.at(idx).used = true;

    return idx;
}

// Update all internal lists after changing the structure.
void CFiniteElementProblem::updateAll()
{
    updateElementsOrientation();
    updateListDoFs();
    updateNodesMainOrientation();
}

void CFiniteElementProblem::updateElementsOrientation()
{
    // Compute global orientation of elements:
    const size_t nElements = m_elements.size();
    for (size_t i = 0; i < nElements; i++)
        m_elements[i]->updateOrientationFromNodePositions();
}

void CFiniteElementProblem::updateNodesMainOrientation()
{
    // Define the per-node main orientation:
    const size_t nNodes = m_node_poses.size();
    m_nodeMainDirection.assign(nNodes, TRotation3D());

    // Take the FIRST element touching this node:
    for (size_t nodeIdx = 0; nodeIdx < nNodes; nodeIdx++)
    {
        const TNodeConnections& ncs = m_node_connections.at(nodeIdx);
        ASSERT_(!ncs.empty());
        const element_index_t elIdx = ncs.begin()->first;
        TRotation3D rot = m_elements.at(elIdx)->getGlobalOrientation();

        // If the element is connected to this node, but it is not the FIRST
        // element "port"/"face", we need to reverse the orientation:
        if (m_elements.at(elIdx)->conected_nodes_ids.at(0) != nodeIdx)
            rot.setRot(rot.getRot() * TRotation3D(0.0, 0.0, M_PI).getRot());

        m_nodeMainDirection.at(nodeIdx) = rot;
    }
}

/** From the list of elements and their properties and connections, build the
 * list of DoFs relevant to the problem.
 */
void CFiniteElementProblem::updateListDoFs()
{
    updateNodeConnections();  // We need these data OK. (m_node_connections)

    {  // Real clear of "m_problem_DoFs"
        std::vector<NodeDoF> dum;
        m_problem_DoFs.swap(dum);
    }

    // Go thru nodes, and add those DoF that appear at least once:
    const size_t nNodes = m_node_poses.size();
    ASSERT_(nNodes == m_node_connections.size());

    // Start with a list of all DOFs marked as unused:
    m_problem_DoFs_inverse_list.assign(nNodes, TProblemDOFIndicesForNode());

    for (size_t nodeIdx = 0; nodeIdx < nNodes; nodeIdx++)
    {
        used_DoFs_t dofs;
        for (unsigned char d = 0; d < 6; d++) dofs[d] = false;

        for (const auto& nc : m_node_connections[nodeIdx])
        {
            for (int d = 0; d < 6; d++)
                if (nc.second.dofs[d]) dofs[d] = true;
        }

        for (uint8_t d = 0; d < 6; d++)
        {
            if (!dofs[d]) continue;

            const int new_dof_index = static_cast<int>(m_problem_DoFs.size());

            m_problem_DoFs.push_back(NodeDoF(nodeIdx, d));
            m_problem_DoFs_inverse_list[nodeIdx].dof_index[d] = new_dof_index;
        }
    }
}

/** From the list of elements, update m_node_connections */
void CFiniteElementProblem::updateNodeConnections()
{
    // Reset list of info per node:
    const size_t nElements = m_elements.size();
    const size_t nNodes    = m_node_poses.size();

    m_node_connections.assign(nNodes, TNodeConnections());

    for (size_t i = 0; i < nElements; i++)
    {
        const auto   el            = m_elements[i];
        const size_t nElementFaces = el->conected_nodes_ids.size();

        vector<used_DoFs_t> el_dofs;
        // el->getLocalDoFs(el_dofs);
        el->getGlobalDoFs(el_dofs);

        for (size_t k = 0; k < nElementFaces; k++)
        {
            ASSERT_(el->conected_nodes_ids[k] < nNodes);
            TNodeConnections& nc =
                m_node_connections[el->conected_nodes_ids[k]];
            nc[i].elementFaceId = static_cast<unsigned char>(k);
            nc[i].dofs          = el_dofs[k];
        }
    }
}

// Report all the problem DOFs as text:
std::string CFiniteElementProblem::getProblemDoFsDescription()
{
    std::stringstream s;

    updateListDoFs();
    for (size_t i = 0; i < m_problem_DoFs.size(); i++)
    {
        s << "DoF #" << i << ": nodeID= " << m_problem_DoFs[i].nodeId
          << " DoF=";
        switch (m_problem_DoFs[i].dof)
        {
            case DoF_index::DX:
                s << "x\n";
                break;
            case DoF_index::DY:
                s << "y\n";
                break;
            case DoF_index::DZ:
                s << "z\n";
                break;
            case DoF_index::RX:
                s << "rotx\n";
                break;
            case DoF_index::RY:
                s << "roty\n";
                break;
            case DoF_index::RZ:
                s << "rotz\n";
                break;
            default:
                break;
        }
    }

    return s.str();
}

/** Return the complementary list of DOFs, such as "return \CUP ds = 1:N" and
 * "return \CAP ds = \empty".
 */
std::vector<size_t> CFiniteElementProblem::complementaryDoFs(
    const std::vector<size_t>& ds, const size_t nTotalDOFs)
{
    // Convert input list into a set:
    std::set<size_t> sv;
    for (size_t i = 0; i < ds.size(); i++) sv.insert(ds[i]);

    std::vector<size_t> v;
    v.reserve(nTotalDOFs - sv.size());

    for (size_t i = 0; i < nTotalDOFs; i++)
        if (sv.find(i) == sv.end()) v.push_back(i);

    return v;
}

/** Returns the 0-based index of the n'th DOF (0-5) of node "nNode" in the list
 * of DOFs to consider in the problem. Callable only after \a buildAll().
 *  Returns string::npos if the DOF is NOT considered in the problem.
 */
size_t CFiniteElementProblem::getDOFIndex(
    const size_t nNode, const DoF_index n) const
{
    int idx =
        m_problem_DoFs_inverse_list[nNode].dof_index[static_cast<uint8_t>(n)];
    if (idx < 0)
        return std::string::npos;
    else
        return static_cast<size_t>(idx);
}

/** Returns the 3D location of the final deformed state after solving the
 * problem */
void CFiniteElementProblem::getNodeDeformedPosition(
    size_t i, Vector3& out_final_point,
    const StaticSolveProblemInfo& solver_info,
    const num_t                   exageration_factor) const
{
    ASSERT_(i < m_node_poses.size());

    Vector3 incr_pt;  // Increment in each (x,y,z) coordinate for this node:

    const TProblemDOFIndicesForNode& DOFs_i =
        this->m_problem_DoFs_inverse_list[i];
    for (int l = 0; l < 3; l++)  // go thru DOFs: x, y, z
    {
        if (DOFs_i.dof_index[l] < 0)
        {
            incr_pt[l] = 0;  // DOF not considered in the problem.
        }
        else
        {
            const size_t dof_index = DOFs_i.dof_index[l];
            // Only one of the indices will be valid:
            const size_t b_idx =
                solver_info.build_info.dof_types[dof_index].bounded_index;
            const size_t f_idx =
                solver_info.build_info.dof_types[dof_index].free_index;

            if (b_idx == std::string::npos)
                incr_pt[l] = solver_info.U_f[f_idx];
            else
                incr_pt[l] = solver_info.build_info.U_b[b_idx];
        }
    }

    // Compose position =============
    // Rotation of increment:
    const TRotationTrans3D& original_pose = m_node_poses[i];
    out_final_point = original_pose.r.getRot() * incr_pt * exageration_factor;

    // Plus original translation:
    for (int i = 0; i < 3; i++) out_final_point[i] += original_pose.t[i];
}

/** Returns the maximum absolute value translation in any X,Y,Z directions
     for a static problem solution */
num_t CFiniteElementProblem::getMaximumDeformedDisplacement(
    const StaticSolveProblemInfo& solver_info) const
{
    num_t cur_max = 0;

    for (size_t i = 0; i < m_problem_DoFs_inverse_list.size(); i++)
    {
        const TProblemDOFIndicesForNode& DOFs_i =
            this->m_problem_DoFs_inverse_list[i];
        for (int l = 0; l < 3; l++)  // go thru DOFs: x, y, z
        {
            if (DOFs_i.dof_index[l] < 0)
                continue;  // DOF not considered in the problem.

            const size_t dof_index = DOFs_i.dof_index[l];
            // Only one of the indices will be valid:
            const size_t b_idx =
                solver_info.build_info.dof_types[dof_index].bounded_index;
            const size_t f_idx =
                solver_info.build_info.dof_types[dof_index].free_index;

            num_t displ;
            if (b_idx == std::string::npos)
                displ = solver_info.U_f[f_idx];
            else
                displ = solver_info.build_info.U_b[b_idx];

            displ = std::abs(displ);
            if (displ > cur_max) cur_max = displ;
        }
    }
    return cur_max;
}

/** Computes the bounding box of all existing nodes */
void CFiniteElementProblem::getBoundingBox(
    num_t& min_x, num_t& max_x, num_t& min_y, num_t& max_y, bool deformed,
    const StaticSolveProblemInfo* solver_info,
    num_t                         deformed_scale_factor) const
{
    ASSERT_(!deformed || solver_info != nullptr);

    min_x = min_y = std::numeric_limits<num_t>::max();
    max_x = max_y = -std::numeric_limits<num_t>::max();

    const size_t nNodes = getNumberOfNodes();
    for (size_t i = 0; i < nNodes; i++)
    {
        if (!deformed)
        {
            const TRotationTrans3D& p = this->getNodePose(i);
            min_x                     = std::min(min_x, p.t.x);
            max_x                     = std::max(max_x, p.t.x);

            min_y = std::min(min_y, p.t.y);
            max_y = std::max(max_y, p.t.y);
        }
        else
        {
            Vector3 pt;
            this->getNodeDeformedPosition(
                i, pt, *solver_info, deformed_scale_factor);
            min_x = std::min(min_x, pt[0]);
            max_x = std::max(max_x, pt[0]);

            min_y = std::min(min_y, pt[1]);
            max_y = std::max(max_y, pt[1]);
        }
    }
}

std::string CFiniteElementProblem::getNodeLabel(const size_t idx) const
{
    if (idx >= m_node_labels.size() || m_node_labels[idx].empty())
    { return openbeam::format("N%u", static_cast<unsigned int>(idx)); }
    else
        return m_node_labels[idx];
}
