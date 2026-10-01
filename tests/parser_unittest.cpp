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

// Structure definition parser: valid inputs, and clear errors for bad ones.

#include <openbeam/openbeam.h>

#include <sstream>

#include "test_helpers.h"

using namespace openbeam;
using namespace openbeam::test;

namespace
{
struct ParseResult
{
    bool            ok = false;
    vector_string_t errors;
    vector_string_t warnings;

    /// All error messages joined, for easy substring checks
    std::string allErrors() const
    {
        std::string s;
        for (const auto& e : errors)
        {
            s += e + "\n";
        }
        return s;
    }
};

ParseResult parse(const std::string& yamlText)
{
    CStructureProblem  problem;
    ParseResult        r;
    std::istringstream is(yamlText);
    r.ok = problem.loadFromStream(is, r.errors, r.warnings);
    return r;
}

const std::string VALID = R"(
parameters:
  L: 2.0
  H: L/2
beam_sections:
- {name: S, E: 2.1e11, A: 1e-3, Iz: 2e-6}
nodes:
- {id: 0, coords: [0, 0], label: A}
- {id: 1, coords: [L, H]}
elements:
- {type: BEAM2D_RR, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDYRZ}
node_loads:
- {node: 1, dof: DY, value: -1000}
)";

/// Replaces the first occurrence of `from` with `to` in VALID
std::string validWith(const std::string& from, const std::string& to)
{
    std::string s   = VALID;
    const auto  pos = s.find(from);
    EXPECT_NE(pos, std::string::npos) << from;
    return s.replace(pos, from.size(), to);
}

}  // namespace

TEST(Parser, ValidDefinition)
{
    CStructureProblem  problem;
    vector_string_t    errors;
    vector_string_t    warnings;
    std::istringstream is(VALID);
    ASSERT_TRUE(problem.loadFromStream(is, errors, warnings));
    EXPECT_TRUE(errors.empty());
    EXPECT_TRUE(warnings.empty());
    EXPECT_EQ(problem.getNumberOfNodes(), 2U);
    EXPECT_EQ(problem.getNumberOfElements(), 1U);
    EXPECT_EQ(problem.getNodeLabel(0), "A");
    // Parameters may use previously defined ones:
    EXPECT_DOUBLE_EQ(problem.getNodePose(1).t.y, 1.0);
}

TEST(Parser, NodesMayBeListedOutOfOrder)
{
    const auto r = parse(validWith(
        "- {id: 0, coords: [0, 0], label: A}\n- {id: 1, coords: [L, H]}",
        "- {id: 1, coords: [L, H]}\n- {id: 0, coords: [0, 0], label: A}"));
    EXPECT_TRUE(r.ok) << r.allErrors();
}

TEST(Parser, DirectionDefaultsAndNormalization)
{
    // DZ omitted, and a non-unit direction vector: both accepted
    auto def = VALID + R"(
element_loads:
- {element: 0, type: DISTRIB_UNIFORM, q: 100, DX: 0, DY: -2}
)";
    const auto r = parse(def);
    EXPECT_TRUE(r.ok) << r.allErrors();
}

TEST(Parser, Errors)
{
    struct Case
    {
        std::string yaml;
        std::string expectedMsg;
    };
    const Case cases[] = {
        {"[1, 2, 3]", "root element must be a map"},
        {validWith("beam_sections:", "sections:"), "beam_sections"},
        {validWith("{id: 1, coords: [L, H]}", "{id: 3, coords: [L, H]}"),
         "Undefined node IDs: 1 2"},
        {validWith("{id: 1, coords: [L, H]}", "{id: 0, coords: [L, H]}"),
         "already defined"},
        {validWith("coords: [L, H]", "coords: [L]"), "coords"},
        {validWith("BEAM2D_RR", "BEAM_FOO"), "Unknown element type 'BEAM_FOO'"},
        {validWith("nodes: [0, 1]", "nodes: [0, 1, 2]"), "expects a list of 2"},
        {validWith("nodes: [0, 1]", "nodes: [0, 7]"), "undefined node ID 7"},
        {validWith("section: S}", "section: X}"), "section 'X'"},
        {validWith("dof: DXDYRZ", "dof: DQ"), "invalid value='DQ'"},
        {validWith("{node: 0, dof: DXDYRZ}", "{node: 5, dof: DXDYRZ}"),
         "undefined node ID 5"},
        {validWith("value: -1000", "value: -1000*Z"),
         "Line 15: Undefined symbol: 'Z' in expression '-1000*Z'"},
        {validWith("  H: L/2", "  H: L/2\n  L: 3"), "already defined"},
        {VALID + "element_loads:\n- {element: 3, type: TEMPERATURE, deltaT: 1}",
         "undefined element index 3"},
        {VALID + "element_loads:\n- {element: 0, type: WIND}",
         "Unknown element load type='WIND'"},
        {VALID + "element_loads:\n- {element: 0, type: DISTRIB_UNIFORM, q: 1, "
                 "DX: 0, DY: 0}",
         "non-null vector"},
        {validWith("{node: 1, dof: DY, value: -1000}", "{node: 1, dof: DY}"),
         "Missing required 'value'"},
    };

    for (const auto& c : cases)
    {
        const auto r = parse(c.yaml);
        EXPECT_FALSE(r.ok) << "Expected failure for:\n" << c.yaml;
        EXPECT_NE(r.allErrors().find(c.expectedMsg), std::string::npos)
            << "Expected message containing: '" << c.expectedMsg
            << "'\nGot:\n"
            << r.allErrors() << "\nInput:\n"
            << c.yaml;
    }
}

TEST(Parser, WarnsAboutIgnoredLoads)
{
    // A truss element has no rotational DoF: a moment on its node is ignored
    const auto r = parse(R"(
beam_sections:
- {name: S, E: 2.1e11, A: 1e-3, Iz: 2e-6}
nodes:
- {id: 0, coords: [0, 0]}
- {id: 1, coords: [1, 0]}
elements:
- {type: BEAM2D_AA, nodes: [0, 1], section: S}
constraints:
- {node: 0, dof: DXDY}
- {node: 1, dof: DY}
node_loads:
- {node: 1, dof: RZ, value: 10}
)");
    EXPECT_TRUE(r.ok) << r.allErrors();
    bool found = false;
    for (const auto& w : r.warnings)
    {
        found = found || w.find("Line 13: (Warning) Load ignored") == 0;
    }
    EXPECT_TRUE(found);
}

TEST(Parser, ReloadClearsPreviousProblem)
{
    CStructureProblem problem;
    for (int i = 0; i < 2; i++)
    {
        std::istringstream is(VALID);
        vector_string_t    errors;
        ASSERT_TRUE(problem.loadFromStream(is, errors));
        EXPECT_EQ(problem.getNumberOfNodes(), 2U);
        EXPECT_EQ(problem.getNumberOfElements(), 1U);
    }
}
