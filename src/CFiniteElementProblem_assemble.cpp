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
void CFiniteElementProblem::assembleProblem(TBuildProblemInfo& out_info)
{
    timelog.enter("assembleProblem");

    updateElementsOrientation();
    updateNodeConnections();
    updateListDoFs();  // Update: m_problem_DoFs

    const size_t nDOFs     = m_problem_DoFs.size();
    const size_t nNodes    = getNumberOfNodes();
    const size_t nElements = m_elements.size();
    OBASSERT(nDOFs > 0)

    // Shortcuts:
    Eigen::SparseMatrix<num_t>& K_bb = out_info.K_bb;
    Eigen::SparseMatrix<num_t>& K_ff = out_info.K_ff;
    Eigen::SparseMatrix<num_t>& K_bf = out_info.K_bf;

    vector<size_t>& bounded_dof_indices = out_info.bounded_dof_indices;
    vector<size_t>& free_dof_indices    = out_info.free_dof_indices;
    vector<TBuildProblemInfo::TDoFType>& dof_types = out_info.dof_types;

    map<size_t, size_t>
        problem_dof2bounded_dof_indices;  // Index in "m_problem_DoFs" to index
                                          // in "bounded_dof_indices"
    map<size_t, size_t>
        problem_dof2free_dof_indices;  // Index in "m_problem_DoFs" to index in
                                       // "free_dof_indices"

    // Create list of free and constrained DoFs from "m_DoF_constraints":
    free_dof_indices.clear();
    bounded_dof_indices.clear();
    dof_types.resize(nDOFs);

    OB_TODO("Do more efficient without find()")
    for (size_t k = 0; k < nDOFs; k++)
    {
        TListOfConstraints::const_iterator it = m_DoF_constraints.find(k);
        if (it == m_DoF_constraints.end())
        {  // Free DOF:
            const size_t new_idx = free_dof_indices.size();
            problem_dof2free_dof_indices.insert(
                problem_dof2free_dof_indices.end(), std::make_pair(k, new_idx));
            free_dof_indices.push_back(k);

            dof_types[k].bounded_index = string::npos;
            dof_types[k].free_index    = new_idx;
        }
        else
        {  // Bounded DOF:
            const size_t new_idx = bounded_dof_indices.size();
            problem_dof2bounded_dof_indices.insert(
                problem_dof2bounded_dof_indices.end(),
                std::make_pair(k, new_idx));
            bounded_dof_indices.push_back(k);

            dof_types[k].bounded_index = new_idx;
            dof_types[k].free_index    = string::npos;
        }
    }

    OB_MESSAGE(2) << nDOFs
                  << " DOFs in the problem: " << free_dof_indices.size()
                  << " free, " << bounded_dof_indices.size()
                  << " constrained.\n";
    OB_MESSAGE(2) << "List of all DOFs:\n" << getProblemDoFsDescription();

    // Start with a dynamic-sparse matrix, later on we'll convert it into real
    // sparse:
    typedef SparseMatrix<num_t>::Index index_t;
    const size_t                       estimated_number_of_non_zero =
        nNodes * 6 * 6 *
        (1 + 1);  // Will store only the UPPER triangular part of K.

    std::vector<Eigen::Triplet<double>> K_tri;
    K_tri.reserve(estimated_number_of_non_zero);

    // Go thru all elements, and get their matrices:
    for (size_t e = 0; e < nElements; e++)
    {
        CElement* el = m_elements[e];

        // Get the matrix as a set of submatrices:
        openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t mats;
        el->getGlobalStiffnessMatrices(mats);

        // Place each submatrix in its place in K:
        for (size_t j = 0; j < mats.size(); j++)
        {
            // Each submatrix model the i->j stiffness:
            const TMatrix66& K_element_org = mats[j].matrix;
            const size_t i_node_idx = el->conected_nodes_ids[mats[j].edge_in];
            const size_t j_node_idx = el->conected_nodes_ids[mats[j].edge_out];

            // Apply nodal coordinates?
            TMatrix66        K_element_nodal;
            const TMatrix66* K_element =
                NULL;  // Will point to either K_element_org or K_element_nodal

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
                K_element       = &K_element_nodal;
                K_element_nodal = K_element_org;
                if (!rt_i.r.isIdentity())
                {
                    K_element_nodal.block<3, 3>(0, 0) =
                        rt_i.r.getRot().transpose() *
                        K_element_nodal.block<3, 3>(0, 0);
                    K_element_nodal.block<3, 3>(0, 3) =
                        rt_i.r.getRot().transpose() *
                        K_element_nodal.block<3, 3>(0, 3);
                    K_element_nodal.block<3, 3>(3, 0) =
                        rt_i.r.getRot().transpose() *
                        K_element_nodal.block<3, 3>(3, 0);
                    K_element_nodal.block<3, 3>(3, 3) =
                        rt_i.r.getRot().transpose() *
                        K_element_nodal.block<3, 3>(3, 3);
                }
                if (!rt_j.r.isIdentity())
                {
                    K_element_nodal.block<3, 3>(0, 0) =
                        K_element_nodal.block<3, 3>(0, 0) * rt_j.r.getRot();
                    K_element_nodal.block<3, 3>(0, 3) =
                        K_element_nodal.block<3, 3>(0, 3) * rt_j.r.getRot();
                    K_element_nodal.block<3, 3>(3, 0) =
                        K_element_nodal.block<3, 3>(3, 0) * rt_j.r.getRot();
                    K_element_nodal.block<3, 3>(3, 3) =
                        K_element_nodal.block<3, 3>(3, 3) * rt_j.r.getRot();
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
            OBASSERT(nc_i.end() != it_nod_i)
            OBASSERT(nc_j.end() != it_nod_j)
            OBASSERT(it_nod_i->second.element_face_id == mats[j].edge_in)
            OBASSERT(it_nod_j->second.element_face_id == mats[j].edge_out)

            const array<int, 6>& mat_rowidx2DoFidx =
                m_problem_DoFs_inverse_list[i_node_idx].dof_index;
            const array<int, 6>& mat_colidx2DoFidx =
                m_problem_DoFs_inverse_list[j_node_idx].dof_index;

            // Send the 6x6 elements in "K_element" to their places, if used:
            for (int r = 0; r < 6; r++)
            {
                const int r_idx_in_K = mat_rowidx2DoFidx[r];
                if (r_idx_in_K < 0) continue;
                for (int c =
                         (i_node_idx == j_node_idx
                              ? r
                              : 0);  // Diagonal blocks: c=[r,5] -> Only upper
                                     // half
                     c < 6; c++)
                {
                    const int c_idx_in_K = mat_colidx2DoFidx[c];
                    if (c_idx_in_K < 0) continue;
                    // Only upper part of K:
                    if (r_idx_in_K > c_idx_in_K)
                        K_tri.push_back(Eigen::Triplet<double>(
                            c_idx_in_K, r_idx_in_K, K_element->coeff(r, c)));
                    else
                        K_tri.push_back(Eigen::Triplet<double>(
                            r_idx_in_K, c_idx_in_K, K_element->coeff(r, c)));
                }
            }  // end insert this 6x6 submatrix

        }  // end for each submatrix
    }  // end for each element

    // Build sparse K from triplets:
    SparseMatrix<num_t> K(
        static_cast<index_t>(nDOFs), static_cast<index_t>(nDOFs));
    K.setFromTriplets(K_tri.begin(), K_tri.end());

    if (openbeam::getVerbosityLevel() >= 3)
        OB_MESSAGE(3) << "Complete K:\n" << Eigen::MatrixX(K) << std::endl;

    // Split K into bounded(R) and free (L) dofs:
    // ------------------------------------------------
    // List of free indices : free_dof_indices
    // Indices are wrt      : m_problem_DoFs

    const index_t nDOFs_f = static_cast<index_t>(free_dof_indices.size());
    const index_t nDOFs_b = static_cast<index_t>(bounded_dof_indices.size());
    OBASSERT(nDOFs_f + nDOFs_b == nDOFs)

    SparseMatrix<num_t> K_bb_aux(nDOFs_b, nDOFs_b);
    SparseMatrix<num_t> K_ff_aux(nDOFs_f, nDOFs_f);
    SparseMatrix<num_t> K_bf_aux(nDOFs_b, nDOFs_f);

    // Symmetric K_bb
    for (index_t r = 0; r < nDOFs_b; r++)
        for (index_t c = r; c < nDOFs_b; c++)
            K_bb_aux.insert(r, c) = K.coeff(
                static_cast<index_t>(bounded_dof_indices[r]),
                static_cast<index_t>(bounded_dof_indices[c]));

    // Symmetric K_ff
    for (index_t r = 0; r < nDOFs_f; r++)
        for (index_t c = r; c < nDOFs_f; c++)
            K_ff_aux.insert(r, c) = K.coeff(
                static_cast<index_t>(free_dof_indices[r]),
                static_cast<index_t>(free_dof_indices[c]));

    // Non-symmetric K_bf
    for (index_t r = 0; r < nDOFs_b; r++)
        for (index_t c = 0; c < nDOFs_f; c++)
        {
            const index_t i_b = static_cast<index_t>(bounded_dof_indices[r]);
            const index_t i_f = static_cast<index_t>(free_dof_indices[c]);

            K_bf_aux.insert(r, c) =
                (i_f >= i_b)
                    ? K.coeff(i_b, i_f)
                    : K.coeff(
                          i_f, i_b);  // Only the UPPER part of K is populated.
        }

    // Convert matrices to the sparse compressed form:
    // -----------------------------------------------------
    K_bb = Eigen::SparseMatrix<num_t>(K_bb_aux);
    K_ff = Eigen::SparseMatrix<num_t>(K_ff_aux);
    K_bf = Eigen::SparseMatrix<num_t>(K_bf_aux);

    // Build U_b vector:
    {
        int idx = 0;
        out_info.U_b.resize(m_DoF_constraints.size());
        for (TListOfConstraints::const_iterator it = m_DoF_constraints.begin();
             it != m_DoF_constraints.end(); ++it)
        {
            OBASSERT(it->first == out_info.bounded_dof_indices[idx])
            out_info.U_b[idx++] = it->second;
        }
    }

    // Before building the list of forces, give children classes the opportunity
    // of
    //  processing special loads:
    this->internalComputeStressAndEquivalentLoads();

    // Build F_f vector:
    // -------------------------
    {
        out_info.F_f.resize(nDOFs_f);
        out_info.F_f.setZero();

        for (size_t i = 0; i < nDOFs_f; i++)
        {
            const size_t dofIdx = out_info.free_dof_indices[i];
            {
                TListOfLoads::const_iterator it =
                    m_loads_at_each_dof.find(dofIdx);
                if (it != m_loads_at_each_dof.end())
                    out_info.F_f[i] = it->second;
            }
            {
                TListOfLoads::const_iterator it =
                    m_loads_at_each_dof_equivs.find(dofIdx);
                if (it != m_loads_at_each_dof_equivs.end())
                    out_info.F_f[i] += it->second;
            }
        }
    }

    // Process nodal coordinates:
    for (size_t i = 0; i < nNodes; i++)
    {
        const TRotationTrans3D& rt_i = this->getNodePose(i);
        if (rt_i.r.isIdentity()) continue;  // Nothing to do here.

        // Loads (Forces and moments) at "i".
        num_t*                     Fs[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
        Eigen::Matrix<num_t, 3, 1> Fi, Mi;
        Fi.setZero();
        Mi.setZero();

        // Bounded (translations and rotations) at "i".
        num_t*                     Us[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
        Eigen::Matrix<num_t, 3, 1> Ui, URi;
        Ui.setZero();
        URi.setZero();

        // make list of pointers once:
        const TProblemDOFIndicesForNode& dof_i = m_problem_DoFs_inverse_list[i];
        for (int k = 0; k < 6; k++)
        {
            if (dof_i.dof_index[k] != -1)
            {
                // This DOF is studied in the problem:
                map<size_t, size_t>::const_iterator it =
                    problem_dof2free_dof_indices.find(dof_i.dof_index[k]);
                if (it != problem_dof2free_dof_indices.end())
                {
                    // it is a free DOF:
                    Fs[k] = &out_info.F_f[it->second];
                }
                else
                {
                    // It must be bounded:
                    map<size_t, size_t>::const_iterator it2 =
                        problem_dof2bounded_dof_indices.find(
                            dof_i.dof_index[k]);
                    OBASSERT(it2 != problem_dof2bounded_dof_indices.end())
                    Us[k] = &out_info.U_b[it2->second];
                }
            }
        }

        // load current values (in global coords) into the temp. vectors:
        for (int k = 0; k < 6; k++)
        {
            if (Fs[k])
            {
                if (k < 3)
                    Fi(k, 0) = *Fs[k];
                else
                    Mi(k - 3, 0) = *Fs[k];
            }
            if (Us[k])
            {
                if (k < 3)
                    Ui(k, 0) = *Us[k];
                else
                    URi(k - 3, 0) = *Us[k];
            }
        }

        // Convert from "global" to "nodal" coordinates:
        Fi = rt_i.r.getRot().transpose() * Fi;
        Mi = rt_i.r.getRot().transpose() * Mi;

        Ui  = rt_i.r.getRot().transpose() * Ui;
        URi = rt_i.r.getRot().transpose() * URi;

        // and save back to the pointers:
        for (int k = 0; k < 6; k++)
        {
            if (Fs[k])
            {
                if (k < 3)
                    *Fs[k] = Fi(k, 0);
                else
                    *Fs[k] = Mi(k - 3, 0);
            }
            if (Us[k])
            {
                if (k < 3)
                    *Us[k] = Ui(k, 0);
                else
                    *Us[k] = URi(k - 3, 0);
            }
        }

    }  // end for each node: process nodal coordinates:

    timelog.leave("assembleProblem");
}
