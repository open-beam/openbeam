#include "AppOpenBeam.h"

#include <localization.h>  // Internationalization support
#include <openbeam/openbeam.h>
#include <openbeam/print_html_matrix.h>

#include <iostream>

std::string AppOpenBeam::LoadStructureDefinition(const std::string& def)
{
    std::string retStr;

    // std::ss << "[Debug] Parsing structure definition:\n" << def <<
    // std::endl;

    // openbeam::setVerbosityLevel(arg_verbose_level.getValue());

    // Load from file:
    openbeam::vector_string errMsg, warnMsg;

    std::stringstream ss(def);
    structure_.loadFromStream(ss, &errMsg, &warnMsg);

    // Return errors:
    for (const auto& m : errMsg)
    {
        retStr += "ERR: ";
        retStr += m;
        retStr += "\n";
    }
    for (const auto& m : warnMsg)
    {
        retStr += "WARN: ";
        retStr += m;
        retStr += "\n";
    }

    return retStr;
}

std::string AppOpenBeam::Solve()
{
    using namespace openbeam;
    using namespace std::string_literals;

    std::string ret;

    TMeshParams mesh_params;
    mesh_params.max_element_length = 0.10;  // [m]

    bool doMesh = false;

    if (doMesh)
    {
        // Mesh:
        structure_.mesh(problem_mesh_, mesh_out_info_, mesh_params);
        problem_to_solve_ = &problem_mesh_;
        mesh_info_        = &mesh_out_info_;
    }
    else
    {
        // Don't mesh:
        problem_to_solve_ = &structure_;
    }

    problem_to_solve_->solveStatic(sInfo_);

    TBuildProblemInfo& info = sInfo_.build_info;
    problem_to_solve_->assembleProblem(info);

    // Stats:
    const size_t nF   = info.free_dof_indices.size();
    const size_t nB   = info.bounded_dof_indices.size();
    const size_t nTot = nF + nB;

    const std::vector<TDoF>& dofs = problem_to_solve_->getProblemDoFs();
    OBASSERT(nTot == dofs.size())

    ret += "Free DOF count: "s + std::to_string(nF) + "\n";
    ret += "Constrained DOF count: "s + std::to_string(nB) + "\n";

    return ret;
}

std::string AppOpenBeam::GetReactionsAsHTML()
{
    using namespace openbeam;
    using namespace openbeam::localization;

    TBuildProblemInfo&       info = sInfo_.build_info;
    const size_t             nF   = info.free_dof_indices.size();
    const size_t             nB   = info.bounded_dof_indices.size();
    const std::vector<TDoF>& dofs = problem_to_solve_->getProblemDoFs();

    std::stringstream ss;
    ss << "<h3>"
       << "(F<sub>R</sub>):</h3>\n";

    ss << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";
    ss << "<tr><td bgcolor=\"#E0E0E0\">" << _t(STR_dof)
       << "</td> <td bgcolor=\"#E0E0E0\">(N, Nm)</td></tr>\n";

    for (size_t i = 0; i < nB; i++)
    {
        const TDoF& dof = dofs[info.bounded_dof_indices[i]];
        ss << "<tr><td>";
        switch (dof.dof)
        {
            case 0:
                ss << "F<sub>x";
                break;
            case 1:
                ss << "F<sub>y";
                break;
            case 2:
                ss << "F<sub>z";
                break;
            case 3:
                ss << "M<sub>x";
                break;
            case 4:
                ss << "M<sub>y";
                break;
            case 5:
                ss << "M<sub>z";
                break;
        };
        ss << dof.node_id;
        ss << "</sub>";
        ss << "</td><td>";
        ss << format("%.2f", sInfo_.F_b[i]);
        ss << "</td></tr>\n";
    }

    ss << "</table>\n";

    return ss.str();
}
