#include "AppOpenBeam.h"

#include <openbeam/openbeam.h>
#include <openbeam/print_html_matrix.h>

#include <iostream>

std::string AppOpenBeam::LoadStructureDefinition(const std::string& def)
{
    std::string retStr;

    // std::cout << "[Debug] Parsing structure definition:\n" << def <<
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

void AppOpenBeam::SayHello()
{
    //
    std::cout << "Hello!" << std::endl;
}
