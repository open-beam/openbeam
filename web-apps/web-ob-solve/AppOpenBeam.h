#pragma once

#include <emscripten/bind.h>
#include <emscripten/html5.h>
#include <openbeam/CStructureProblem.h>

namespace e = emscripten;

class AppOpenBeam
{
   public:
    /** Returns: error descriptions. Empty string if it was all good. */
    std::string LoadStructureDefinition(const std::string& def);

    /** Returns: log messages */
    std::string Solve();

    std::string GetReactionsAsHTML();
    std::string GetDisplacementsAsHTML();

   private:
    openbeam::CStructureProblem structure_;

    openbeam::CStructureProblem      problem_mesh_;
    openbeam::MeshOutputInfo         mesh_out_info_;
    openbeam::CStructureProblem*     problem_to_solve_ = nullptr;
    openbeam::MeshOutputInfo*        mesh_info_        = nullptr;
    openbeam::StaticSolveProblemInfo sInfo_;
};

EMSCRIPTEN_BINDINGS(EMTest)
{
    e::class_<AppOpenBeam>("AppOpenBeam")
        .constructor()
        .function(
            "LoadStructureDefinition", &AppOpenBeam::LoadStructureDefinition)
        .function("Solve", &AppOpenBeam::Solve)
        .function("GetReactionsAsHTML", &AppOpenBeam::GetReactionsAsHTML)
        .function(
            "GetDisplacementsAsHTML", &AppOpenBeam::GetDisplacementsAsHTML);
}
