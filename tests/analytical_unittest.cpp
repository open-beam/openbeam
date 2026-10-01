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

// Solutions checked against closed-form results from beam theory.

#include <cmath>

#include "test_helpers.h"

using namespace openbeam;
using namespace openbeam::test;

namespace
{
// Common section: E (Pa), A (m^2), Iz (m^4)
constexpr double E  = 2.1e11;
constexpr double A  = 1e-3;
constexpr double Iz = 2e-6;
constexpr double EI = E * Iz;
constexpr double EA = E * A;

constexpr double TOL = 1e-9;  // relative

const std::string SECTION = R"(
parameters:
  L: 3.0
  P: 1000
  q: 2000
beam_sections:
- {name: S, E: 2.1e11, A: 1e-3, Iz: 2e-6}
)";

}  // namespace

TEST(Analytical, CantileverTipLoad)
{
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
node_loads:
- {node: 1, dof: DY, value: -P}
)");
    const double L = 3.0;
    const double P = 1000;

    EXPECT_REL_NEAR(
        displacement(s, 1, DoF_index::DY), -P * L * L * L / (3 * EI), TOL);
    EXPECT_REL_NEAR(displacement(s, 1, DoF_index::RZ), -P * L * L / (2 * EI), TOL);
    EXPECT_NEAR(displacement(s, 1, DoF_index::DX), 0, 1e-15);

    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), P, TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::RZ), P * L, TOL);
    EXPECT_NEAR(reaction(s, 0, DoF_index::DX), 0, 1e-9);

    // Bending moment: maximum (P*L) at the fixed end, zero at the free end
    const auto& st = s.stress.element_stress.at(0);
    EXPECT_REL_NEAR(std::abs(st.at(0).Mz), P * L, TOL);
    EXPECT_NEAR(st.at(1).Mz, 0, 1e-9);
    EXPECT_REL_NEAR(std::abs(st.at(0).Vy), P, TOL);
}

TEST(Analytical, AxialBar)
{
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_AA, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 1, dof: DY}
node_loads:
- {node: 1, dof: DX, value: P}
)");
    const double L = 3.0;
    const double P = 1000;
    EXPECT_REL_NEAR(displacement(s, 1, DoF_index::DX), P * L / EA, TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DX), -P, TOL);
    // Tension
    EXPECT_REL_NEAR(std::abs(s.stress.element_stress.at(0).at(0).N), P, TOL);
}

TEST(Analytical, SimplySupportedUniformLoad)
{
    // Two elements, so there is a node at mid-span. Beam elements with
    // consistent nodal loads give exact nodal displacements.
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L/2, 0]}
- {id: 2, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
- {type: BEAM2D_RR, nodes: [1, 2], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 2, dof: DY}
element_loads:
- {element: 0, type: DISTRIB_UNIFORM, q: q, DX: 0, DY: -1}
- {element: 1, type: DISTRIB_UNIFORM, q: q, DX: 0, DY: -1}
)");
    const double L = 3.0;
    const double q = 2000;

    EXPECT_REL_NEAR(
        displacement(s, 1, DoF_index::DY), -5 * q * std::pow(L, 4) / (384 * EI),
        TOL);
    EXPECT_REL_NEAR(
        displacement(s, 0, DoF_index::RZ), -q * std::pow(L, 3) / (24 * EI), TOL);
    EXPECT_REL_NEAR(
        displacement(s, 2, DoF_index::RZ), q * std::pow(L, 3) / (24 * EI), TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), q * L / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 2, DoF_index::DY), q * L / 2, TOL);

    // Maximum bending moment at mid-span: qL^2/8
    const auto& st = s.stress.element_stress;
    EXPECT_REL_NEAR(std::abs(st.at(0).at(1).Mz), q * L * L / 8, TOL);
    EXPECT_REL_NEAR(std::abs(st.at(1).at(0).Mz), q * L * L / 8, TOL);
    // No shear at mid-span
    EXPECT_NEAR(st.at(0).at(1).Vy, 0, 1e-6);
}

TEST(Analytical, FixedFixedUniformLoad)
{
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
- {node: 1, dof: DXDYRZ}
element_loads:
- {element: 0, type: DISTRIB_UNIFORM, q: q, DX: 0, DY: -1}
)");
    const double L = 3.0;
    const double q = 2000;
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), q * L / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::DY), q * L / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::RZ), q * L * L / 12, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::RZ), -q * L * L / 12, TOL);

    const auto& st = s.stress.element_stress.at(0);
    EXPECT_REL_NEAR(std::abs(st.at(0).Mz), q * L * L / 12, TOL);
    EXPECT_REL_NEAR(std::abs(st.at(1).Mz), q * L * L / 12, TOL);
}

TEST(Analytical, SimplySupportedConcentratedLoad)
{
    // Point load P at distance a from the left support: R_left = P*b/L
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 1, dof: DY}
element_loads:
- {element: 0, type: CONCENTRATED, p: P, dist: 1.0, DX: 0, DY: -1}
)");
    const double L = 3.0;
    const double P = 1000;
    const double a = 1.0;
    const double b = L - a;
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), P * b / L, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::DY), P * a / L, TOL);
    // End rotation: theta_A = P*a*b*(L+b)/(6*E*I*L)
    EXPECT_REL_NEAR(
        displacement(s, 0, DoF_index::RZ), -P * a * b * (L + b) / (6 * EI * L),
        TOL);
}

TEST(Analytical, SimplySupportedTriangularLoad)
{
    // Load growing linearly from 0 to q: reactions qL/6 and qL/3
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 1, dof: DY}
element_loads:
- {element: 0, type: TRIANGULAR, q_ini: 0, q_end: q, DX: 0, DY: -1}
)");
    const double L = 3.0;
    const double q = 2000;
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), q * L / 6, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::DY), q * L / 3, TOL);
    // End rotations: 7qL^3/(360EI) and 8qL^3/(360EI)
    EXPECT_REL_NEAR(
        displacement(s, 0, DoF_index::RZ), -7 * q * std::pow(L, 3) / (360 * EI),
        TOL);
    EXPECT_REL_NEAR(
        displacement(s, 1, DoF_index::RZ), 8 * q * std::pow(L, 3) / (360 * EI),
        TOL);
}

TEST(Analytical, TwoBarTruss)
{
    // Symmetric truss: supports at (0,0) and (2,0), load P down at (1,1).
    // Each bar carries P/sqrt(2) in compression.
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [2, 0]}
- {id: 2, coords: [1, 1]}
elements:
- {type: BEAM2D_AA, nodes: [0, 2], section: S}
- {type: BEAM2D_AA, nodes: [1, 2], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 1, dof: DXDY}
node_loads:
- {node: 2, dof: DY, value: -P}
)");
    const double P = 1000;
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DY), P / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::DY), P / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DX), P / 2, TOL);
    EXPECT_REL_NEAR(reaction(s, 1, DoF_index::DX), -P / 2, TOL);
    for (int e = 0; e < 2; e++)
    {
        EXPECT_REL_NEAR(
            s.stress.element_stress.at(e).at(0).N, -P / std::sqrt(2.0), TOL);
    }
    // Vertical deflection of the apex: 2 * N^2 * Lbar / (E A P)
    const double Lbar = std::sqrt(2.0);
    EXPECT_REL_NEAR(displacement(s, 2, DoF_index::DY), -P * Lbar / EA, TOL);
    EXPECT_NEAR(displacement(s, 2, DoF_index::DX), 0, 1e-15);
}

TEST(Analytical, PropppedCantileverSupportSettlement)
{
    // Fixed at the left, roller at the right which settles down by delta:
    // the roller reaction is 3*E*I*delta/L^3 (pulling down).
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
- {node: 1, dof: DY, value: -0.01}
)");
    const double L     = 3.0;
    const double delta = 0.01;
    EXPECT_REL_NEAR(displacement(s, 1, DoF_index::DY), -delta, TOL);
    EXPECT_REL_NEAR(
        reaction(s, 1, DoF_index::DY), -3 * EI * delta / (L * L * L), TOL);
    EXPECT_REL_NEAR(
        reaction(s, 0, DoF_index::DY), 3 * EI * delta / (L * L * L), TOL);
    EXPECT_REL_NEAR(
        displacement(s, 1, DoF_index::RZ), -3 * delta / (2 * L), TOL);
}

TEST(Analytical, SettlementOfTheFirstSupport)
{
    // Same as above, mirrored: the settling support is node 0, so the
    // prescribed displacement couples to constrained DoFs numbered after it.
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DY, value: -0.01}
- {node: 1, dof: DXDYRZ}
)");
    const double L     = 3.0;
    const double delta = 0.01;
    EXPECT_REL_NEAR(
        reaction(s, 0, DoF_index::DY), -3 * EI * delta / (L * L * L), TOL);
    EXPECT_REL_NEAR(
        reaction(s, 1, DoF_index::DY), 3 * EI * delta / (L * L * L), TOL);
    // Moment balance about node 1: M1 = -(-L) * R0
    EXPECT_REL_NEAR(
        reaction(s, 1, DoF_index::RZ), -3 * EI * delta / (L * L), TOL);
}

TEST(Analytical, SpringsInSeries)
{
    const auto s = solveYaml(R"(
beam_sections: []
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [1, 0]}
- {id: 2, coords: [2, 0]}
elements:
- {type: SPRING_1D, nodes: [0, 1], K: 1000}
- {type: SPRING_1D, nodes: [1, 2], K: 4000}
constraints:
- {node: 0, dof: DX}
node_loads:
- {node: 2, dof: DX, value: 100}
)");
    EXPECT_REL_NEAR(displacement(s, 1, DoF_index::DX), 100.0 / 1000, TOL);
    EXPECT_REL_NEAR(
        displacement(s, 2, DoF_index::DX), 100.0 / 1000 + 100.0 / 4000, TOL);
    EXPECT_REL_NEAR(reaction(s, 0, DoF_index::DX), -100, TOL);
}

TEST(Analytical, FixedBarHeated)
{
    // A restrained bar does not move; its axial force is E*A*alpha*dT
    // (compression), with the default steel alpha = 12e-6.
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
- {node: 1, dof: DXDYRZ}
element_loads:
- {element: 0, type: TEMPERATURE, deltaT: 20}
)");
    const double N = EA * 12e-6 * 20;
    for (int f = 0; f < 2; f++)
    {
        EXPECT_REL_NEAR(s.stress.element_stress.at(0).at(f).N, -N, TOL);
    }
    EXPECT_REL_NEAR(std::abs(reaction(s, 0, DoF_index::DX)), N, TOL);
}

TEST(Analytical, FreeBarHeated)
{
    // A bar free to expand gets longer by alpha*dT*L and carries no force.
    const auto s = solveYaml(SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
element_loads:
- {element: 0, type: TEMPERATURE, deltaT: 20}
)");
    const double L = 3.0;
    EXPECT_REL_NEAR(displacement(s, 1, DoF_index::DX), 12e-6 * 20 * L, TOL);
    for (int f = 0; f < 2; f++)
    {
        EXPECT_NEAR(s.stress.element_stress.at(0).at(f).N, 0, 1e-6);
    }
}

TEST(Analytical, DenseAndSparseSolversAgree)
{
    const std::string def = SECTION + R"(
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [0, L]}
- {id: 2, coords: [L, L]}
- {id: 3, coords: [L, 0]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
- {type: BEAM2D_RR, nodes: [1, 2], section: S}
- {type: BEAM2D_RR, nodes: [2, 3], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
- {node: 3, dof: DXDY}
node_loads:
- {node: 1, dof: DX, value: P}
element_loads:
- {element: 1, type: DISTRIB_UNIFORM, q: q, DX: 0, DY: -1}
)";
    const auto dense  = solveYaml(def, StaticSolverAlgorithm::Dense_LLT);
    const auto sparse = solveYaml(def, StaticSolverAlgorithm::Sparse_LLT);
    ASSERT_EQ(dense.info.U_f.size(), sparse.info.U_f.size());
    for (int i = 0; i < dense.info.U_f.size(); i++)
    {
        EXPECT_REL_NEAR(sparse.info.U_f[i], dense.info.U_f[i], 1e-9);
    }
    for (int i = 0; i < dense.info.F_b.size(); i++)
    {
        EXPECT_REL_NEAR(sparse.info.F_b[i], dense.info.F_b[i], 1e-9);
    }
}
