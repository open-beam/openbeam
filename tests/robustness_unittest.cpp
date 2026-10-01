/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2026  Jose Luis Blanco Claraco                       |
   |                                                                           |
   | OpenBeam is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   +---------------------------------------------------------------------------+
 */

// Badly posed structures (mechanisms, ill-conditioning) and large problems.

#include <chrono>
#include <sstream>

#include "test_helpers.h"

using namespace openbeam;
using namespace openbeam::test;

namespace
{
const std::string SECTION = R"(
beam_sections:
- {name: S, E: 2.1e11, A: 1e-3, Iz: 2e-6}
)";

/** Parses a definition (which must be valid) and returns the solver error
 * message, or an empty string if it solved fine. */
std::string solveError(
    const std::string& yamlText, StaticSolverAlgorithm algorithm = StaticSolverAlgorithm::Dense_LLT)
{
  CStructureProblem problem;
  vector_string_t errors;
  std::istringstream is(yamlText);
  EXPECT_TRUE(problem.loadFromStream(is, errors));
  try
  {
    StaticSolveProblemInfo info;
    StaticSolverOptions opts;
    opts.algorithm = algorithm;
    problem.solveStatic(info, opts);
    return {};
  }
  catch (const std::exception& e)
  {
    return e.what();
  }
}

/** A rectangular frame of `bays` x `stories` rigid beams, fixed at the base,
 * with horizontal loads at each floor and distributed loads on the floors. */
std::string frameDefinition(int bays, int stories)
{
  std::ostringstream s;
  s << SECTION << "nodes:\n";
  const auto id = [bays](int i, int j) { return j * (bays + 1) + i; };
  for (int j = 0; j <= stories; j++)
  {
    for (int i = 0; i <= bays; i++)
    {
      s << "- {id: " << id(i, j) << ", coords: [" << 4 * i << ", " << 3 * j << "]}\n";
    }
  }
  s << "elements:\n";
  int nElements = 0;
  std::vector<int> floorBeams;
  for (int j = 0; j < stories; j++)
  {
    for (int i = 0; i <= bays; i++)  // columns
    {
      s << "- {type: BEAM2D_RR, nodes: [" << id(i, j) << ", " << id(i, j + 1) << "], section: S}\n";
      nElements++;
    }
    for (int i = 0; i < bays; i++)  // floor beams
    {
      s << "- {type: BEAM2D_RR, nodes: [" << id(i, j + 1) << ", " << id(i + 1, j + 1)
        << "], section: S}\n";
      floorBeams.push_back(nElements++);
    }
  }
  s << "constraints:\n";
  for (int i = 0; i <= bays; i++)
  {
    s << "- {node: " << id(i, 0) << ", dof: DXDYRZ}\n";
  }
  s << "node_loads:\n";
  for (int j = 1; j <= stories; j++)
  {
    s << "- {node: " << id(0, j) << ", dof: DX, value: 1000}\n";
  }
  s << "element_loads:\n";
  for (int e : floorBeams)
  {
    s << "- {element: " << e << ", type: DISTRIB_UNIFORM, q: 5000, DX: 0, DY: -1}\n";
  }
  return s.str();
}

}  // namespace

TEST(Robustness, MechanismIsReported)
{
  // A beam on two rollers: it can slide horizontally.
  const auto msg = solveError(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0], label: A}
- {id: 1, coords: [3, 0], label: B}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DY}
- {node: 1, dof: DY}
)");
  EXPECT_NE(msg.find("unstable"), std::string::npos) << msg;
  EXPECT_NE(msg.find("A DX"), std::string::npos) << msg;
  EXPECT_NE(msg.find("B DX"), std::string::npos) << msg;
}

TEST(Robustness, MechanismIsReportedBySparseSolver)
{
  const auto msg = solveError(
      SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [3, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDY}
)",
      StaticSolverAlgorithm::Sparse_LLT);
  EXPECT_NE(msg.find("unstable"), std::string::npos) << msg;
}

TEST(Robustness, NegativeStiffnessIsReported)
{
  const auto msg = solveError(R"(
beam_sections:
- {name: S, E: -2.1e11, A: 1e-3, Iz: 2e-6}
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [3, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
)");
  EXPECT_NE(msg.find("not positive definite"), std::string::npos) << msg;
}

TEST(Robustness, IllConditioningWarning)
{
  // A spring far stiffer than the beam it connects to:
  const std::string def = SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [3, 0]}
- {id: 2, coords: [4, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
- {type: SPRING_1D, nodes: [1, 2], K: 1e19}
constraints:
- {node: 0, dof: DXDYRZ}
node_loads:
- {node: 2, dof: DX, value: 1}
)";
  for (const auto algo : {StaticSolverAlgorithm::Dense_LLT, StaticSolverAlgorithm::Sparse_LLT})
  {
    const auto s = solveYaml(def, algo);
    EXPECT_LT(s.info.rcond, 1e-12);
    ASSERT_EQ(s.info.warnings.size(), 1U);
    EXPECT_NE(s.info.warnings[0].find("ill-conditioned"), std::string::npos);
  }
}

TEST(Robustness, WellConditionedHasNoWarning)
{
  const auto s = solveYaml(frameDefinition(3, 3));
  EXPECT_GT(s.info.rcond, 1e-12);
  EXPECT_TRUE(s.info.warnings.empty());
}

TEST(Robustness, FullVectorsAreFilled)
{
  const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [3, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
node_loads:
- {node: 1, dof: DY, value: -1000}
)");
  const auto nDOFs = s.info.build_info.dof_types.size();
  ASSERT_EQ(static_cast<size_t>(s.info.U.size()), nDOFs);
  ASSERT_EQ(static_cast<size_t>(s.info.F.size()), nDOFs);
  const auto iDY1 = s.problem->getDOFIndex(1, DoF_index::DY);
  const auto iDY0 = s.problem->getDOFIndex(0, DoF_index::DY);
  EXPECT_DOUBLE_EQ(s.info.U[iDY1], displacement(s, 1, DoF_index::DY));
  EXPECT_DOUBLE_EQ(s.info.F[iDY1], -1000);
  EXPECT_DOUBLE_EQ(s.info.F[iDY0], reaction(s, 0, DoF_index::DY));
  EXPECT_DOUBLE_EQ(s.info.U[iDY0], 0);
}

TEST(Robustness, LargeFrameSparseVsDense)
{
  // ~750 DoFs: both solvers must agree
  const auto def = frameDefinition(20, 12);
  const auto dense = solveYaml(def, StaticSolverAlgorithm::Dense_LLT);
  const auto sparse = solveYaml(def, StaticSolverAlgorithm::Sparse_LLT);
  ASSERT_EQ(dense.info.U_f.size(), sparse.info.U_f.size());
  EXPECT_GT(dense.info.U_f.size(), 700);
  const double scale = dense.info.U_f.cwiseAbs().maxCoeff();
  EXPECT_LT((dense.info.U_f - sparse.info.U_f).cwiseAbs().maxCoeff(), 1e-9 * scale);
  EXPECT_TRUE(dense.info.warnings.empty());

  // Global equilibrium: vertical reactions balance the floor loads
  double sumFy = 0;
  for (int i = 0; i <= 20; i++)
  {
    sumFy += reaction(sparse, i, DoF_index::DY);
  }
  EXPECT_REL_NEAR(sumFy, 5000.0 * 4 * 20 * 12, 1e-9);
}

TEST(Robustness, VeryLargeFrameSparse)
{
  // ~10,000 DoFs with the sparse solver must stay fast
  const auto def = frameDefinition(40, 80);
  const auto t0 = std::chrono::steady_clock::now();
  const auto s = solveYaml(def, StaticSolverAlgorithm::Sparse_LLT);
  const auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GT(s.info.U_f.size(), 9000);
  EXPECT_TRUE(s.info.U_f.array().isFinite().all());
  std::cout << "VeryLargeFrameSparse: " << s.info.U_f.size() << " free DoFs solved in " << dt
            << " s\n";
  EXPECT_LT(dt, 10.0);
}
