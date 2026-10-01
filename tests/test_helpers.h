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

#pragma once

#include <gtest/gtest.h>
#include <openbeam/openbeam.h>

#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace openbeam::test
{
/** A parsed and solved structure. The problem is heap allocated since elements
 * keep a pointer to their parent problem. */
struct Solution
{
    std::unique_ptr<CStructureProblem> problem =
        std::make_unique<CStructureProblem>();
    StaticSolveProblemInfo info;
    StressInfo             stress;
    vector_string_t        errors;
    vector_string_t        warnings;
};

/** Parses a structure definition (YAML text) and runs the static solver and
 * stress post-processing. Adds a test failure on parse errors. */
inline Solution solveYaml(
    const std::string&    yamlText,
    StaticSolverAlgorithm algorithm = StaticSolverAlgorithm::Dense_LLT)
{
    Solution           s;
    std::istringstream is(yamlText);
    const bool         ok = s.problem->loadFromStream(is, s.errors, s.warnings);
    EXPECT_TRUE(ok) << "Parse errors:\n"
                    << (s.errors.empty() ? std::string() : s.errors.at(0));
    if (!ok)
    {
        return s;
    }
    StaticSolverOptions opts;
    opts.algorithm = algorithm;
    s.problem->solveStatic(s.info, opts);
    s.problem->postProcCalcStress(s.stress, s.info);
    return s;
}

/** Displacement of a node DoF (in nodal coordinates), whether free or
 * constrained. */
inline num_t displacement(const Solution& s, size_t node, DoF_index dof)
{
    const size_t idx = s.problem->getDOFIndex(node, dof);
    if (idx == std::string::npos)
    {
        ADD_FAILURE() << "DoF not in problem: node " << node;
        return std::numeric_limits<num_t>::quiet_NaN();
    }
    const auto& t = s.info.build_info.dof_types.at(idx);
    if (t.free_index != std::string::npos)
    {
        return s.info.U_f[t.free_index];
    }
    return s.info.build_info.U_b[t.bounded_index];
}

/** Reaction force (or moment) at a constrained node DoF; zero for free DoFs. */
inline num_t reaction(const Solution& s, size_t node, DoF_index dof)
{
    const size_t idx = s.problem->getDOFIndex(node, dof);
    if (idx == std::string::npos)
    {
        ADD_FAILURE() << "DoF not in problem: node " << node;
        return std::numeric_limits<num_t>::quiet_NaN();
    }
    const auto& t = s.info.build_info.dof_types.at(idx);
    if (t.bounded_index == std::string::npos)
    {
        return 0;
    }
    return s.info.F_b[t.bounded_index];
}

/// Relative tolerance check: |a-b| <= tol * max(|b|, tiny)
#define EXPECT_REL_NEAR(a, b, tol)                                        \
    EXPECT_NEAR(                                                          \
        (a), (b),                                                         \
        (tol) * std::max(std::abs(static_cast<double>(b)), 1e-12))

}  // namespace openbeam::test
