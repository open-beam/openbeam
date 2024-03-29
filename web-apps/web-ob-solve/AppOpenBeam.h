#pragma once

#include <emscripten/bind.h>
#include <emscripten/html5.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <openbeam/CStructureProblem.h>

namespace e = emscripten;

class AppOpenBeam
{
   public:
    AppOpenBeam();

    /** Returns: error descriptions. Empty string if it was all good. */
    std::string LoadStructureDefinition(const std::string& def);

    bool HasValidStructure() const { return builtOk_; }

    /** Returns: log messages
     *
     *  options, as YAML/JSON:
     *  \code
     *  mesh: true
     *  mesh_max_length: 0.10
     *  \endcode
     */
    std::string Solve(const std::string& options);

    std::string GetReactionsAsHTML();
    std::string GetDisplacementsAsHTML();
    std::string GetStressAsHTML();

    void generateVisualization(const std::string& options);
    void repaintCanvas();

    double determineAutoDeformationScale();

   private:
    openbeam::CStructureProblem structure_;
    bool                        builtOk_ = false;

    openbeam::CStructureProblem      problem_mesh_;
    openbeam::MeshOutputInfo         mesh_out_info_;
    openbeam::CStructureProblem*     problem_to_solve_ = nullptr;
    openbeam::MeshOutputInfo*        mesh_info_        = nullptr;
    openbeam::StaticSolveProblemInfo sInfo_;
    openbeam::StressInfo             stressInfo_;

    mrpt::opengl::COpenGLScene::Ptr theScene_;
};

EMSCRIPTEN_BINDINGS(EMTest)
{
    e::class_<AppOpenBeam>("AppOpenBeam")
        .constructor()
        .function("repaintCanvas", &AppOpenBeam::repaintCanvas)
        .function(
            "LoadStructureDefinition", &AppOpenBeam::LoadStructureDefinition)
        .function("Solve", &AppOpenBeam::Solve)
        .function("generateVisualization", &AppOpenBeam::generateVisualization)
        .function("GetReactionsAsHTML", &AppOpenBeam::GetReactionsAsHTML)
        .function(
            "GetDisplacementsAsHTML", &AppOpenBeam::GetDisplacementsAsHTML)
        .function("GetStressAsHTML", &AppOpenBeam::GetStressAsHTML)
        .function("HasValidStructure", &AppOpenBeam::HasValidStructure)
        .function(
            "determineAutoDeformationScale",
            &AppOpenBeam::determineAutoDeformationScale);
}
