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

using namespace std;
using namespace openbeam;
using namespace Eigen;

template <class K_DECOMP>
void solveStaticInternal(
    const K_DECOMP& K_decomp, StaticSolveProblemInfo& out_info,
    const bool U_b_all_zeros)
{
    // Uf = Kff^-1 * Ff
    out_info.U_f = K_decomp.solve(out_info.build_info.F_f);

    if (!U_b_all_zeros)
    {
        const DynMatrix Kbf = DynMatrix(out_info.build_info.K_bf);
        // Uf = (Kff^-1 * Ff)  -   Kff^-1 * Kbf * Ub
        out_info.U_f -=
            K_decomp.solve(Kbf.transpose() * out_info.build_info.U_b);
    }
}

// ------------------------------------------------------------------------------------------
// Solve the complete FE problem, returning the resulting displacements and
// reaction forces.
// ------------------------------------------------------------------------------------------
void CFiniteElementProblem::solveStatic(
    StaticSolveProblemInfo& out_info, const StaticSolverOptions& opts)
{
    mrpt::system::CTimeLoggerEntry tle(openbeam::timelog, "solveStatic");

    // If doing iterations, save the initial location of all nodes:
    std::deque<TRotationTrans3D> orig_nodes;
    if (opts.nonLinearIterative) { orig_nodes = this->m_node_poses; }

    bool U_b_all_zeros = false;

    // Iteration loop: will do only ONE iteration for linear, small-deformation
    // problems:
    unsigned int nMaxIters = 40;
    unsigned int nIter     = 0;
    while (nIter < nMaxIters)
    {
        nIter++;

        // Build stiffness matrices & vectors of restrictions and loads:
        // (TODO: Make more efficient by only recomputing what really changes)
        this->assembleProblem(out_info.build_info);

        // Check if all U_b != 0 to use simplified expressions:
        U_b_all_zeros = (out_info.build_info.U_b.array() == 0).all();

        DynMatrix Kff = DynMatrix(out_info.build_info.K_ff.transpose());

        if (opts.algorithm == StaticSolverAlgorithm::SVD)
        {  // make sym:
            const int N = static_cast<int>(Kff.rows());
            for (int r = 0; r < N; r++)
                for (int c = r + 1; c < N; c++)
                    Kff.coeffRef(r, c) = Kff.coeffRef(c, r);
        }

        OB_MESSAGE(3) << "SolveStatic (iter=" << nIter
                      << ") will try to decompose matrix Kff:\n"
                      << Kff << endl;

        // Decomposition of a positive definite matrix for fast solving Ax=b
        // below:
        switch (opts.algorithm)
        {
            case StaticSolverAlgorithm::LLT:
            {
                const Eigen::LLT<DynMatrix> Kff_llt = Kff.llt();
                if (!Kff_llt.info() == Eigen::Success)
                {
                    // There's a structure problem! Probably it's not a
                    // structure, but a mechanism...
                    throw std::runtime_error(
                        "[CFiniteElementProblem::solveStatic] Ill-conditioned "
                        "matrix. Too few restrictions?");
                }

                // Do solve:
                solveStaticInternal(Kff_llt, out_info, U_b_all_zeros);
            }
            break;

            case StaticSolverAlgorithm::SVD:
            {
                const Eigen::JacobiSVD<DynMatrix> Kff_llt(
                    Kff, Eigen::ComputeThinU | Eigen::ComputeThinV);

                // Do solve:
                solveStaticInternal(Kff_llt, out_info, U_b_all_zeros);
            }
            break;

            default:
                throw std::runtime_error("Invalid solver algorithm value");
                break;
        };

        // If this is iterative: check ending condition and update node poses.
        if (opts.nonLinearIterative)
        {
            const num_t change_norm_L2 = std::sqrt(
                out_info.U_f.array().square().sum() / out_info.U_f.size());
            OB_MESSAGE(1) << "[CFiniteElementProblem::solveStatic] Iter "
                          << nIter << ", change_norm_L2 = " << change_norm_L2
                          << endl;

            for (size_t i = 0; i < out_info.build_info.free_dof_indices.size();
                 i++)
            {
                const num_t delta_val =
                    out_info.U_f[i];  // The increment for this DoF

                const size_t dof_idx = out_info.build_info.free_dof_indices[i];
                const NodeDoF& dof   = m_problem_DoFs[dof_idx];

                if (dof.dofAsInt() < 3)
                {
                    // X, Y, Z
                    m_node_poses[dof.nodeId].t[dof.dofAsInt()] += delta_val;
                }
                else
                {
                    // A rotation:
                    num_t rx = 0, ry = 0, rz = 0;
                    if (dof.dof == DoF_index::RX)
                        rx = delta_val;
                    else if (dof.dof == DoF_index::RY)
                        ry = delta_val;
                    else if (dof.dof == DoF_index::RZ)
                        rz = delta_val;

                    m_node_poses[dof.nodeId].r.setRot(
                        m_node_poses[dof.nodeId].r.getRot() *
                        TRotation3D(rx, ry, rz).getRot());
                }
            }
        }
        else
        {
            break;  // Only one iteration is enough for linear problems with
                    // small displacements.
        }

    }  // end iterative loops

    // If doing iterations, the U_f displacements are actually only wrt the
    // contents of "m_node_poses":
    //  Recompute overall displacements and restore the back-up of node poses:
    if (opts.nonLinearIterative)
    {
        this->m_node_poses = orig_nodes;  // Restore copy
    }

    // Next, reactions:
    if (!U_b_all_zeros)
        out_info.F_b = out_info.build_info.K_bb * out_info.build_info.U_b +
                       out_info.build_info.K_bf * out_info.U_f;
    else
        out_info.F_b = out_info.build_info.K_bf * out_info.U_f;

    // Add the contributions of distributed forces to the reactions:

    // typedef std::map<size_t,num_t> TListOfLoads;
    // m_loads_at_each_dof_equivs: The vector of overall loads (F_L') on each
    // DoF due to distributed forces
    //  Map keys are indices of \a m_problem_DoFs
    for (load_list_t::const_iterator it = m_loads_at_each_dof_equivs.begin();
         it != m_loads_at_each_dof_equivs.end(); ++it)
    {
        const size_t idx_dof = it->first;
        const double F_value = it->second;

        const size_t idx_restricted =
            out_info.build_info.dof_types[idx_dof].bounded_index;

        if (idx_restricted != std::string::npos)
            out_info.F_b[idx_restricted] -= F_value;
    }
}
