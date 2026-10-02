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

#include <gtest/gtest.h>

#include "test_helpers.h"

using namespace openbeam;
using namespace openbeam::test;

namespace
{
// A frame with a roller at C. Modeled twice: with C as a plain node, and
// with C rotated 90 degrees (so its local X is the global Y).
std::string frame(
    const std::string& nodeC, const std::string& constraintC, const std::string& loadC)
{
  return R"(
beam_sections:
- {name: S, E: 2.1e11, A: 1e-3, Iz: 2e-6}
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [0, 2]}
- )" + nodeC +
         R"(
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
- {type: BEAM2D_RR, nodes: [1, 2], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
- )" + constraintC +
         R"(
node_loads:
- {node: 1, dof: DX, value: 1000}
- )" + loadC +
         R"(
element_loads:
- {element: 1, type: DISTRIB_UNIFORM, q: 500, DX: 0, DY: -1}
)";
}
}  // namespace

TEST(RotatedNodes, SameResultsAsGlobalAxes)
{
  const auto a = solveYaml(
      frame("{id: 2, coords: [3, 2]}", "{node: 2, dof: DY}", "{node: 2, dof: DX, value: 300}"));
  const auto b = solveYaml(frame(
      "{id: 2, coords: [3, 2], rot_z: 90}", "{node: 2, dof: DX}",
      "{node: 2, dof: DX, value: 300}"));
  ASSERT_TRUE(a.errors.empty());
  ASSERT_TRUE(b.errors.empty());

  // Same internal forces at both ends of both elements:
  for (size_t e = 0; e < 2; e++)
  {
    for (size_t f = 0; f < 2; f++)
    {
      const auto& sa = a.stress.element_stress.at(e).at(f);
      const auto& sb = b.stress.element_stress.at(e).at(f);
      EXPECT_NEAR(sa.N, sb.N, 1e-6) << "element " << e << " face " << f;
      EXPECT_NEAR(sa.Vy, sb.Vy, 1e-6) << "element " << e << " face " << f;
      EXPECT_NEAR(sa.Mz, sb.Mz, 1e-6) << "element " << e << " face " << f;
    }
  }
  // Same support reaction (nodal X of the rotated node is global Y):
  EXPECT_NEAR(reaction(a, 2, DoF_index::DY), reaction(b, 2, DoF_index::DX), 1e-6);
  // Same displacement of the free direction (nodal Y is global -X):
  EXPECT_NEAR(displacement(a, 2, DoF_index::DX), -displacement(b, 2, DoF_index::DY), 1e-12);
}
