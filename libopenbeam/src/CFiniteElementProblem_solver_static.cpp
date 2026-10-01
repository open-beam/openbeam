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

#include <Eigen/Eigenvalues>
#include <set>

using namespace openbeam;

namespace
{
/// Below this reciprocal condition number, results are flagged as inaccurate
constexpr num_t RCOND_WARNING_THRESHOLD = 1e-12;

/// Largest problem for which mechanisms are diagnosed with an eigen
/// decomposition (O(n^3)) when the factorization fails.
constexpr Eigen::Index MAX_DOFS_FOR_MECHANISM_DIAGNOSIS = 3000;

const char* dofName(DoF_index d)
{
  switch (d)
  {
    case DoF_index::DX:
      return "DX";
    case DoF_index::DY:
      return "DY";
    case DoF_index::DZ:
      return "DZ";
    case DoF_index::RX:
      return "RX";
    case DoF_index::RY:
      return "RY";
    case DoF_index::RZ:
      return "RZ";
  }
  return "?";
}

template <class K_DECOMP>
void solveStaticInternal(
    const K_DECOMP& K_decomp, StaticSolveProblemInfo& out_info, const bool U_b_all_zeros)
{
  // Uf = Kff^-1 * Ff
  out_info.U_f = K_decomp.solve(out_info.build_info.F_f);

  if (!U_b_all_zeros)
  {
    // Uf = (Kff^-1 * Ff)  -   Kff^-1 * Kbf^T * Ub
    const Eigen::Matrix<num_t, Eigen::Dynamic, 1> KbfT_Ub =
        out_info.build_info.K_bf.transpose() * out_info.build_info.U_b;
    out_info.U_f -= K_decomp.solve(KbfT_Ub);
  }
}

}  // namespace

std::string CFiniteElementProblem::describeSingularStiffness(
    const DynMatrix& Kff, const std::vector<size_t>& free_dof_indices) const
{
  const std::string generic =
      "The stiffness matrix is singular: the structure is probably a "
      "mechanism (some part can move freely). Check the constraints and "
      "the element connections.";

  if (Kff.rows() == 0 || Kff.rows() > MAX_DOFS_FOR_MECHANISM_DIAGNOSIS)
  {
    return generic;
  }

  // Kff is symmetric: build the full matrix from its lower triangle (the
  // only one that assembleProblem() fills in).
  const DynMatrix K = Kff.selfadjointView<Eigen::Lower>();
  const Eigen::SelfAdjointEigenSolver<DynMatrix> es(K);
  if (es.info() != Eigen::Success)
  {
    return generic;
  }

  const auto& ev = es.eigenvalues();  // ascending
  const num_t maxEig = std::max(ev.cwiseAbs().maxCoeff(), num_t(1e-300));
  const num_t tol = 1e-12 * maxEig;

  if (ev[0] < -tol)
  {
    return "The stiffness matrix is not positive definite: check for "
           "negative or zero section properties (E, A, Iz) or spring "
           "constants.";
  }

  // Each eigenvector with a ~0 eigenvalue is a motion with no stiffness.
  // List the DoFs that take part in them:
  std::set<size_t> involved;
  int nModes = 0;
  for (Eigen::Index m = 0; m < ev.size() && ev[m] <= tol; m++)
  {
    nModes++;
    const auto v = es.eigenvectors().col(m);
    const num_t vmax = v.cwiseAbs().maxCoeff();
    for (Eigen::Index i = 0; i < v.size(); i++)
    {
      if (std::abs(v[i]) >= 0.5 * vmax)
      {
        involved.insert(free_dof_indices.at(i));
      }
    }
  }
  if (nModes == 0)
  {
    return "The stiffness matrix is too ill-conditioned to be solved. "
           "Very stiff springs or elements much stiffer than the rest of "
           "the structure usually cause this.";
  }

  std::string dofList;
  for (size_t idx : involved)
  {
    const auto& d = m_problem_DoFs.at(idx);
    if (!dofList.empty())
    {
      dofList += ", ";
    }
    dofList += getNodeLabel(d.nodeId) + " " + dofName(d.dof);
  }
  return mrpt::format(
      "The structure is unstable (a mechanism with %i free motion mode%s): "
      "it can move without resistance at %s. Add constraints or check the "
      "element connections.",
      nModes, nModes > 1 ? "s" : "", dofList.c_str());
}

// ----------------------------------------------------------------------------
// Solve the complete FE problem, returning the resulting displacements and
// reaction forces.
// ----------------------------------------------------------------------------
void CFiniteElementProblem::solveStatic(
    StaticSolveProblemInfo& out_info, const StaticSolverOptions& opts)
{
  mrpt::system::CTimeLoggerEntry tle(openbeam::timelog, "solveStatic");

  out_info.warnings.clear();
  out_info.rcond = std::numeric_limits<num_t>::quiet_NaN();

  // If doing iterations, save the initial location of all nodes:
  std::deque<TRotationTrans3D> orig_nodes;
  if (opts.nonLinearIterative)
  {
    orig_nodes = this->m_node_poses;
  }

  bool U_b_all_zeros = false;

  // Iteration loop: will do only ONE iteration for linear, small-deformation
  // problems:
  const unsigned int nMaxIters = 40;
  unsigned int nIter = 0;
  while (nIter < nMaxIters)
  {
    nIter++;

    mrpt::system::CTimeLoggerEntry tle2(openbeam::timelog, "solveStatic.1_assemble");

    // Build stiffness matrices & vectors of restrictions and loads:
    this->assembleProblem(out_info.build_info);

    tle2.stop();

    mrpt::system::CTimeLoggerEntry tle3(openbeam::timelog, "solveStatic.2_solve");

    // Check if all U_b == 0 to use simplified expressions:
    U_b_all_zeros = (out_info.build_info.U_b.array() == 0).all();

    const auto& free_dofs = out_info.build_info.free_dof_indices;

    if (free_dofs.empty())
    {
      // Everything is constrained: nothing to solve for.
      out_info.U_f.resize(0);
    }
    else if (opts.algorithm == StaticSolverAlgorithm::Dense_LLT)
    {
      const DynMatrix Kff = DynMatrix(out_info.build_info.K_ff);

      OB_MESSAGE(3) << "SolveStatic (iter=" << nIter << ") will try to decompose matrix Kff:\n"
                    << Kff << std::endl;

      // Cholesky decomposition (Kff is symmetric positive definite for
      // a well-constrained structure):
      const Eigen::LLT<DynMatrix> Kff_llt = Kff.llt();
      if (Kff_llt.info() != Eigen::Success)
      {
        throw std::runtime_error(describeSingularStiffness(Kff, free_dofs));
      }
      out_info.rcond = Kff_llt.rcond();
      solveStaticInternal(Kff_llt, out_info, U_b_all_zeros);
    }
    else if (opts.algorithm == StaticSolverAlgorithm::Sparse_LLT)
    {
      openbeam::timelog.enter("solveStatic.2_solve_analyze");

      Eigen::SimplicialLLT<Eigen::SparseMatrix<num_t>> cholesky;
      cholesky.analyzePattern(out_info.build_info.K_ff);

      openbeam::timelog.leave("solveStatic.2_solve_analyze");
      openbeam::timelog.enter("solveStatic.2_solve_factorize");

      cholesky.factorize(out_info.build_info.K_ff);

      openbeam::timelog.leave("solveStatic.2_solve_factorize");

      if (cholesky.info() != Eigen::Success)
      {
        throw std::runtime_error(
            describeSingularStiffness(DynMatrix(out_info.build_info.K_ff), free_dofs));
      }
      // Cheap estimate of the reciprocal condition number from the
      // range of the Cholesky factor diagonal:
      const auto d = cholesky.matrixL().nestedExpression().diagonal();
      const num_t dMax = d.cwiseAbs().maxCoeff();
      out_info.rcond = dMax > 0 ? square(d.cwiseAbs().minCoeff() / dMax) : 0;
      solveStaticInternal(cholesky, out_info, U_b_all_zeros);
    }
    else
    {
      throw std::runtime_error("Invalid solver algorithm value");
    }

    if (!out_info.U_f.array().isFinite().all())
    {
      throw std::runtime_error(
          "The solution contains non-finite values: check for NaN or "
          "infinite values in the structure definition.");
    }

    tle3.stop();

    // If this is iterative: check ending condition and update node
    // poses.
    if (!opts.nonLinearIterative)
    {
      // Only one iteration is enough for linear problems with small
      // displacements.
      break;
    }

    const num_t change_norm_L2 = std::sqrt(
        out_info.U_f.array().square().sum() / std::max<Eigen::Index>(out_info.U_f.size(), 1));
    OB_MESSAGE(1) << "[CFiniteElementProblem::solveStatic] Iter " << nIter
                  << ", change_norm_L2 = " << change_norm_L2 << std::endl;

    for (size_t i = 0; i < free_dofs.size(); i++)
    {
      const num_t delta_val = out_info.U_f[i];
      const NodeDoF& dof = m_problem_DoFs[free_dofs[i]];

      if (dof.dofAsInt() < 3)
      {
        // X, Y, Z
        m_node_poses[dof.nodeId].t[dof.dofAsInt()] += delta_val;
      }
      else
      {
        // A rotation:
        num_t rx = 0;
        num_t ry = 0;
        num_t rz = 0;
        if (dof.dof == DoF_index::RX)
        {
          rx = delta_val;
        }
        else if (dof.dof == DoF_index::RY)
        {
          ry = delta_val;
        }
        else if (dof.dof == DoF_index::RZ)
        {
          rz = delta_val;
        }
        m_node_poses[dof.nodeId].r.setRot(
            m_node_poses[dof.nodeId].r.getRot() * TRotation3D(rx, ry, rz).getRot());
      }
    }
  }  // end iterative loops

  // If doing iterations, the U_f displacements are actually only wrt the
  // contents of "m_node_poses": restore the original node poses.
  if (opts.nonLinearIterative)
  {
    this->m_node_poses = orig_nodes;
  }

  if (out_info.rcond < RCOND_WARNING_THRESHOLD)
  {
    out_info.warnings.push_back(mrpt::format(
        "The stiffness matrix is ill-conditioned (rcond=%g): results may "
        "be inaccurate. Very stiff springs or elements much stiffer than "
        "the rest of the structure usually cause this.",
        out_info.rcond));
  }

  // Reactions:
  const auto& bi = out_info.build_info;
  if (bi.free_dof_indices.empty())
  {
    out_info.F_b = bi.K_bb * bi.U_b;
  }
  else if (!U_b_all_zeros)
  {
    out_info.F_b = bi.K_bb * bi.U_b + bi.K_bf * out_info.U_f;
  }
  else
  {
    out_info.F_b = bi.K_bf * out_info.U_f;
  }

  // Add the contributions of distributed loads to the reactions.
  // m_loads_at_each_dof_equivs: equivalent loads (F_L') on each DoF due to
  // element loads. Map keys are indices of m_problem_DoFs.
  for (const auto& [idx_dof, F_value] : m_loads_at_each_dof_equivs)
  {
    const size_t idx_restricted = bi.dof_types[idx_dof].bounded_index;
    if (idx_restricted != std::string::npos)
    {
      out_info.F_b[idx_restricted] -= F_value;
    }
  }

  // Full U and F vectors, for all DoFs:
  const size_t nDOFs = bi.dof_types.size();
  out_info.U.resize(nDOFs);
  out_info.F.resize(nDOFs);
  for (size_t k = 0; k < nDOFs; k++)
  {
    const auto& t = bi.dof_types[k];
    if (t.free_index != std::string::npos)
    {
      out_info.U[k] = out_info.U_f[t.free_index];
      out_info.F[k] = bi.F_f[t.free_index];
    }
    else
    {
      out_info.U[k] = bi.U_b[t.bounded_index];
      out_info.F[k] = out_info.F_b[t.bounded_index];
    }
  }
}
