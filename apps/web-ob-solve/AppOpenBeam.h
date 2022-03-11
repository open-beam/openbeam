#pragma once

#include <emscripten/bind.h>
#include <emscripten/html5.h>

namespace e = emscripten;

class AppOpenBeam
{
   public:
    void LoadStructureDefinition(const std::string& def);
    void SayHello();
};

EMSCRIPTEN_BINDINGS(EMTest)
{
    e::class_<AppOpenBeam>("AppOpenBeam")
        .constructor()
        .function(
            "LoadStructureDefinition", &AppOpenBeam::LoadStructureDefinition)
        .function("SayHello", &AppOpenBeam::SayHello);
}
