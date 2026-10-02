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

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;

/** For any partitioning of the DoFs into fixed (restricted, boundary
 * conditions) and the rest (the free variables \a free_dof_indices), where DoF
 * indices refer to \a m_problem_DoFs, computes three sparse matrices: K_{FF}
 * (free-free), K_{BB} (boundary-boundary) and K_{BF} (boundary-free) which
 * define the stiffness matrix of the problem.
 */
void CFiniteElementProblem::assembleProblem(BuildProblemInfo& out_info)
{
  auto tle = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem");

  updateElementsOrientation();
  updateNodeConnections();
  updateListDoFs();  // Update: m_problem_DoFs

  const size_t nDOFs = m_problem_DoFs.size();
  const size_t nNodes = getNumberOfNodes();
  const size_t nElements = m_elements.size();
  ASSERT_(nDOFs > 0);

  // Shortcuts:
  Eigen::SparseMatrix<num_t>& K_bb = out_info.K_bb;
  Eigen::SparseMatrix<num_t>& K_ff = out_info.K_ff;
  Eigen::SparseMatrix<num_t>& K_bf = out_info.K_bf;

  vector<size_t>& bounded_dof_indices = out_info.bounded_dof_indices;
  vector<size_t>& free_dof_indices = out_info.free_dof_indices;
  vector<BuildProblemInfo::TDoFType>& dof_types = out_info.dof_types;

  map<size_t, size_t> problem_dof2bounded_dof_indices;  // Index in "m_problem_DoFs" to index
                                                        // in "bounded_dof_indices"
  map<size_t, size_t> problem_dof2free_dof_indices;     // Index in "m_problem_DoFs" to index in
                                                        // "free_dof_indices"

  // Create list of free and constrained DoFs from "m_DoF_constraints":
  free_dof_indices.clear();
  bounded_dof_indices.clear();
  dof_types.resize(nDOFs);

  auto tle2 = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem.1_list_dofs");

  for (size_t k = 0; k < nDOFs; k++)
  {
    if (auto it = m_DoF_constraints.find(k); it == m_DoF_constraints.end())
    {
      // Free DOF:
      const size_t new_idx = free_dof_indices.size();
      problem_dof2free_dof_indices.insert(
          problem_dof2free_dof_indices.end(), std::make_pair(k, new_idx));
      free_dof_indices.push_back(k);

      dof_types[k].bounded_index = string::npos;
      dof_types[k].free_index = new_idx;
    }
    else
    {
      // Constrained DOF:
      const size_t new_idx = bounded_dof_indices.size();
      problem_dof2bounded_dof_indices.insert(
          problem_dof2bounded_dof_indices.end(), std::make_pair(k, new_idx));
      bounded_dof_indices.push_back(k);

      dof_types[k].bounded_index = new_idx;
      dof_types[k].free_index = string::npos;
    }
  }

  tle2.stop();

  OB_MESSAGE(2) << nDOFs << " DOFs in the problem: " << free_dof_indices.size() << " free, "
                << bounded_dof_indices.size() << " constrained.\n";
  OB_MESSAGE(2) << "List of all DOFs:\n" << getProblemDoFsDescription();

  /// Start with a dynamic-sparse matrix, later on we'll convert it into real
  /// sparse:
  using index_t = SparseMatrix<num_t>::Index;

  /// Will store only the UPPER triangular part of K.
  const size_t estimated_number_of_non_zero = nNodes * 6 * 6 * (1 + 1);

  std::vector<Eigen::Triplet<double>> K_tri;
  K_tri.reserve(estimated_number_of_non_zero);

  auto tle3 = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem.2_matrices");

  // Go thru all elements, and get their matrices:
  for (size_t e = 0; e < nElements; e++)
  {
    auto el = m_elements[e];

    // Get the matrix as a set of submatrices:
    std::vector<TStiffnessSubmatrix> mats;
    el->getGlobalStiffnessMatrices(mats);

    // Place each submatrix in its place in K:
    for (size_t j = 0; j < mats.size(); j++)
    {
      // Each submatrix model the i->j stiffness:
      const Matrix66& K_element_org = mats[j].matrix;
      const size_t i_node_idx = el->conected_nodes_ids[mats[j].edge_in];
      const size_t j_node_idx = el->conected_nodes_ids[mats[j].edge_out];

      // Apply nodal coordinates?
      Matrix66 KeNodal;
      // Will point to either K_element_org or K_element_nodal
      const Matrix66* K_element = nullptr;

      const TRotationTrans3D& rt_i = this->getNodePose(i_node_idx);
      const TRotationTrans3D& rt_j = this->getNodePose(j_node_idx);
      if (rt_i.r.isIdentity() && rt_j.r.isIdentity())
      {
        // Use a shortcut for the most common case where the local
        // coordinates coincide with global ones:
        K_element = &K_element_org;
      }
      else
      {
        // Cases: T^t * K  , K * T  or T^t * K * T
        K_element = &KeNodal;
        KeNodal = K_element_org;
        if (!rt_i.r.isIdentity())
        {
          KeNodal.block<3, 3>(0, 0) = rt_i.r.getRot().transpose() * KeNodal.block<3, 3>(0, 0);
          KeNodal.block<3, 3>(0, 3) = rt_i.r.getRot().transpose() * KeNodal.block<3, 3>(0, 3);
          KeNodal.block<3, 3>(3, 0) = rt_i.r.getRot().transpose() * KeNodal.block<3, 3>(3, 0);
          KeNodal.block<3, 3>(3, 3) = rt_i.r.getRot().transpose() * KeNodal.block<3, 3>(3, 3);
        }
        if (!rt_j.r.isIdentity())
        {
          KeNodal.block<3, 3>(0, 0) = KeNodal.block<3, 3>(0, 0) * rt_j.r.getRot();
          KeNodal.block<3, 3>(0, 3) = KeNodal.block<3, 3>(0, 3) * rt_j.r.getRot();
          KeNodal.block<3, 3>(3, 0) = KeNodal.block<3, 3>(3, 0) * rt_j.r.getRot();
          KeNodal.block<3, 3>(3, 3) = KeNodal.block<3, 3>(3, 3) * rt_j.r.getRot();
        }
      }

      // If  i_node_idx == i_node_idx: Sum to the diagonal block matrix.
      //                    Otherwise: Insert into the matrix (we know for
      //                    sure it's new).

      // We need to know which DoFs from all the 6 are really used in the
      // node.
      const TNodeConnections& nc_i = m_node_connections[i_node_idx];
      const TNodeConnections& nc_j = m_node_connections[j_node_idx];

      TNodeConnections::const_iterator it_nod_i = nc_i.find(e);
      TNodeConnections::const_iterator it_nod_j = nc_j.find(e);
      ASSERT_(nc_i.end() != it_nod_i);
      ASSERT_(nc_j.end() != it_nod_j);
      ASSERT_(it_nod_i->second.elementFaceId == mats[j].edge_in);
      ASSERT_(it_nod_j->second.elementFaceId == mats[j].edge_out);

      const array<int, 6>& mat_rowidx2DoFidx = m_problem_DoFs_inverse_list[i_node_idx].dof_index;
      const array<int, 6>& mat_colidx2DoFidx = m_problem_DoFs_inverse_list[j_node_idx].dof_index;

      // Send the 6x6 elements in "K_element" to their places, if used:
      for (int r = 0; r < 6; r++)
      {
        const int r_idx_in_K = mat_rowidx2DoFidx[r];
        if (r_idx_in_K < 0) continue;
        // Diagonal blocks: c=[r,5] -> Only upper half
        for (int c = (i_node_idx == j_node_idx ? r : 0); c < 6; c++)
        {
          const int c_idx_in_K = mat_colidx2DoFidx[c];
          if (c_idx_in_K < 0) continue;

          // Only upper part of K:
          if (r_idx_in_K > c_idx_in_K)
            K_tri.push_back(Eigen::Triplet<double>(c_idx_in_K, r_idx_in_K, K_element->coeff(r, c)));
          else
            K_tri.push_back(Eigen::Triplet<double>(r_idx_in_K, c_idx_in_K, K_element->coeff(r, c)));
        }
      }  // end insert this 6x6 submatrix
    }    // end for each submatrix
  }      // end for each element

  tle3.stop();

  auto tle4 = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem.4_sub_Ks");

  // Split K into its bounded (b) and free (f) blocks, straight from the
  // triplets of its upper triangle:
  //  - K_ff: lower triangle only, as read by the Cholesky solvers.
  //  - K_bb: full symmetric matrix.
  //  - K_bf: full (rectangular) matrix.
  const index_t nDOFs_f = static_cast<index_t>(free_dof_indices.size());
  const index_t nDOFs_b = static_cast<index_t>(bounded_dof_indices.size());
  ASSERT_EQUAL_(static_cast<size_t>(nDOFs_f + nDOFs_b), nDOFs);

  std::vector<Eigen::Triplet<num_t>> tri_ff;
  std::vector<Eigen::Triplet<num_t>> tri_bb;
  std::vector<Eigen::Triplet<num_t>> tri_bf;
  tri_ff.reserve(K_tri.size());

  for (const auto& t : K_tri)
  {
    const auto& tr = dof_types[static_cast<size_t>(t.row())];
    const auto& tc = dof_types[static_cast<size_t>(t.col())];
    const bool rFree = tr.free_index != string::npos;
    const bool cFree = tc.free_index != string::npos;

    if (rFree && cFree)
    {
      const auto i = static_cast<index_t>(tr.free_index);
      const auto j = static_cast<index_t>(tc.free_index);
      tri_ff.emplace_back(std::max(i, j), std::min(i, j), t.value());
    }
    else if (!rFree && !cFree)
    {
      const auto i = static_cast<index_t>(tr.bounded_index);
      const auto j = static_cast<index_t>(tc.bounded_index);
      tri_bb.emplace_back(i, j, t.value());
      if (i != j)
      {
        tri_bb.emplace_back(j, i, t.value());
      }
    }
    else if (rFree)
    {
      tri_bf.emplace_back(
          static_cast<index_t>(tc.bounded_index), static_cast<index_t>(tr.free_index), t.value());
    }
    else
    {
      tri_bf.emplace_back(
          static_cast<index_t>(tr.bounded_index), static_cast<index_t>(tc.free_index), t.value());
    }
  }

  // (Duplicated entries are summed up)
  K_ff.resize(nDOFs_f, nDOFs_f);
  K_ff.setFromTriplets(tri_ff.begin(), tri_ff.end());
  K_bb.resize(nDOFs_b, nDOFs_b);
  K_bb.setFromTriplets(tri_bb.begin(), tri_bb.end());
  K_bf.resize(nDOFs_b, nDOFs_f);
  K_bf.setFromTriplets(tri_bf.begin(), tri_bf.end());

  if (openbeam::getVerbosityLevel() >= 3)
  {
    SparseMatrix<num_t> K(static_cast<index_t>(nDOFs), static_cast<index_t>(nDOFs));
    K.setFromTriplets(K_tri.begin(), K_tri.end());
    OB_MESSAGE(3) << "Complete K (upper triangle):\n" << Eigen::MatrixXd(K) << std::endl;
  }

  tle4.stop();

  auto tle7 = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem.6_Ub");

  // Build U_b vector:
  {
    int idx = 0;
    out_info.U_b.resize(m_DoF_constraints.size());
    for (constraint_list_t::const_iterator it = m_DoF_constraints.begin();
         it != m_DoF_constraints.end(); ++it)
    {
      ASSERT_(it->first == out_info.bounded_dof_indices[idx]);
      out_info.U_b[idx++] = it->second;
    }
  }

  tle7.stop();

  auto tle8 = mrpt::system::CTimeLoggerEntry(timelog, "assembleProblem.7_Ff");

  // Before building the list of forces, give children classes the opportunity
  // of processing special loads:
  this->internalComputeStressAndEquivalentLoads();

  // Applied loads, direct and equivalent ones from element loads, are given in
  // global coordinates. Convert them to nodal coordinates and split them into
  // free DoFs (F_f) and constrained ones (F_b_applied, taken by the supports):
  out_info.F_f.setZero(nDOFs_f);
  out_info.F_b_applied.setZero(nDOFs_b);
  {
    std::vector<Vector6> nodeLoads(nNodes, Vector6::Zero());
    for (const auto* loads : {&m_loads_at_each_dof, &m_loads_at_each_dof_equivs})
    {
      for (const auto& [dofIdx, value] : *loads)
      {
        const NodeDoF& d = m_problem_DoFs.at(dofIdx);
        nodeLoads[d.nodeId][d.dofAsInt()] += value;
      }
    }

    for (size_t i = 0; i < nNodes; i++)
    {
      Vector6 f = nodeLoads[i];
      const TRotation3D& rot = this->getNodePose(i).r;
      if (!rot.isIdentity())
      {
        f.head<3>() = rot.getRot().transpose() * f.head<3>();
        f.tail<3>() = rot.getRot().transpose() * f.tail<3>();
      }
      for (int k = 0; k < 6; k++)
      {
        const int dofIdx = m_problem_DoFs_inverse_list[i].dof_index[k];
        if (f[k] == 0 || dofIdx < 0)
        {
          continue;
        }
        const auto& t = dof_types[static_cast<size_t>(dofIdx)];
        if (t.free_index != string::npos)
        {
          out_info.F_f[t.free_index] += f[k];
        }
        else
        {
          out_info.F_b_applied[t.bounded_index] += f[k];
        }
      }
    }
  }

  tle8.stop();
}
