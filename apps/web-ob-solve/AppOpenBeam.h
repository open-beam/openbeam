#pragma once

#include <emscripten/bind.h>
#include <emscripten/html5.h>

namespace e = emscripten;

class AppOpenBeam
{
   public:
    void Initialize();
    void SayHello();
};

EMSCRIPTEN_BINDINGS(EMTest)
{
    e::class_<AppOpenBeam>("AppOpenBeam")
        .constructor()
        .function("Initialize", &AppOpenBeam::Initialize)
        .function("SayHello", &AppOpenBeam::SayHello);
}
