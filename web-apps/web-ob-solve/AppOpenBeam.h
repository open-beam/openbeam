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

   private:
    openbeam::CStructureProblem structure_;

    openbeam::CStructureProblem       problem_mesh_;
    openbeam::TMeshOutputInfo         mesh_out_info_;
    openbeam::CStructureProblem*      problem_to_solve_ = nullptr;
    openbeam::TMeshOutputInfo*        mesh_info_        = nullptr;
    openbeam::TStaticSolveProblemInfo sInfo_;
};

EMSCRIPTEN_BINDINGS(EMTest)
{
    e::class_<AppOpenBeam>("AppOpenBeam")
        .constructor()
        .function(
            "LoadStructureDefinition", &AppOpenBeam::LoadStructureDefinition)
        .function("Solve", &AppOpenBeam::Solve)
        .function("GetReactionsAsHTML", &AppOpenBeam::GetReactionsAsHTML);
}
