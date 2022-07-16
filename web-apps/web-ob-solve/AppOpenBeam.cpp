

#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
//#include "/home/jlblanco/code/mrpt/3rdparty/nanogui/ext/glfw/deps/linmath.h"
#include <GLFW/glfw3.h>
//
#include <localization.h>  // Internationalization support
#include <mrpt/opengl/CGridPlaneXY.h>
#include <openbeam/openbeam.h>
#include <openbeam/print_html_matrix.h>

#include <cstdio>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>

#include "AppOpenBeam.h"

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}
std::function<void()> loop;
void                  main_loop()
{
    // loop();
}
GLFWwindow* window = nullptr;

// Initialize WebGL
AppOpenBeam::AppOpenBeam()
{
    try
    {
        glfwSetErrorCallback(error_callback);

        if (!glfwInit()) exit(EXIT_FAILURE);

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(800, 400, "Openbeam", nullptr, nullptr);
        if (!window)
        {
            glfwTerminate();
            exit(EXIT_FAILURE);
        }
        // glfwSetKeyCallback(window, key_callback);
        glfwMakeContextCurrent(window);

        glfwSwapInterval(1);

        // TestDisplay3D();

        std::cout << "OpenGL version " << glGetString(GL_VERSION) << std::endl;

        // theScene_->getViewport()->getCamera().setZoomDistance(20);

        theScene_ = mrpt::opengl::COpenGLScene::Create();
        {
            auto obj =
                mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
            obj->setColor(0.8f, 0.8f, 0.8f);
            theScene_->insert(obj);
        }

        // emscripten_set_main_loop(main_loop, 0, true);

        // glfwDestroyWindow(window);
        // glfwTerminate();
        // exit(EXIT_SUCCESS);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
    }
}

void AppOpenBeam::repaintCanvas()
{
    if (!window || !theScene_) return;

    try
    {
        glfwMakeContextCurrent(window);
        glClearColor(0.4f, 0.4f, 0.4f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        // theScene_->getViewport()->getCamera().setPointingAt(camx,
        // camy, camz);

        theScene_->render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

std::string AppOpenBeam::LoadStructureDefinition(const std::string& def)
{
    std::string retStr;

    // std::ss << "[Debug] Parsing structure definition:\n" << def <<
    // std::endl;

    // openbeam::setVerbosityLevel(arg_verbose_level.getValue());

    // Load from file:
    openbeam::vector_string_t errMsg, warnMsg;

    std::stringstream ss(def);
    structure_.loadFromStream(ss, errMsg, warnMsg);

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

    try
    {
        MeshParams mesh_params;
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

        BuildProblemInfo& info = sInfo_.build_info;
        problem_to_solve_->assembleProblem(info);

        // Stats:
        const size_t nF   = info.free_dof_indices.size();
        const size_t nB   = info.bounded_dof_indices.size();
        const size_t nTot = nF + nB;

        const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();
        ASSERT_(nTot == dofs.size());

        ret += "Free DOF count: "s + std::to_string(nF) + "\n";
        ret += "Constrained DOF count: "s + std::to_string(nB) + "\n";
    }
    catch (const std::exception& e)
    {
        ret += e.what();
    }

    return ret;
}

std::string AppOpenBeam::GetReactionsAsHTML()
{
    using namespace openbeam;
    using namespace openbeam::localization;

    BuildProblemInfo&           info = sInfo_.build_info;
    const size_t                nF   = info.free_dof_indices.size();
    const size_t                nB   = info.bounded_dof_indices.size();
    const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();

    std::stringstream ss;
    ss << "<h3>" << _t(STR_Reactions) << " (F<sub>R</sub>):</h3>\n";

    ss << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";
    ss << "<tr><td bgcolor=\"#E0E0E0\">" << _t(STR_dof)
       << "</td> <td bgcolor=\"#E0E0E0\">(N, Nm)</td></tr>\n";

    for (size_t i = 0; i < nB; i++)
    {
        const NodeDoF& dof = dofs[info.bounded_dof_indices[i]];
        ss << "<tr><td>";
        switch (dof.dofAsInt())
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
        ss << dof.nodeId;
        ss << "</sub>";
        ss << "</td><td>";
        ss << format("%.2f", sInfo_.F_b[i]);
        ss << "</td></tr>\n";
    }

    ss << "</table>\n";

    return ss.str();
}

std::string AppOpenBeam::GetDisplacementsAsHTML()
{
    using namespace openbeam;
    using namespace openbeam::localization;

    std::stringstream ss;

    BuildProblemInfo&           info = sInfo_.build_info;
    const size_t                nF   = info.free_dof_indices.size();
    const size_t                nB   = info.bounded_dof_indices.size();
    const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();
    const size_t nTotalNodes         = problem_to_solve_->getNumberOfNodes();

    std::vector<std::vector<double>> U(nTotalNodes);
    for (size_t i = 0; i < nTotalNodes; i++) U[i].assign(6, 0);
    for (size_t i = 0; i < nF; i++)
    {
        const NodeDoF& dof            = dofs[info.free_dof_indices[i]];
        U[dof.nodeId][dof.dofAsInt()] = sInfo_.U_f[i];
    }
    for (size_t i = 0; i < nB; i++)
    {
        const NodeDoF& dof            = dofs[info.bounded_dof_indices[i]];
        U[dof.nodeId][dof.dofAsInt()] = info.U_b[i];
    }

    ss << "<h3>" << _t(STR_Displacements) << " (U<sub>R</sub> " << _t(STR_and)
       << " U<sub>L</sub>):</h3>\n";

    ss << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";

    ss << "<tr>"
          "<td bgcolor=\"#E0E0E0\">"
       << _t(STR_node)
       << "</td>"
          "<td bgcolor=\"#E0E0E0\">DX (mm)</td>"
          "<td bgcolor=\"#E0E0E0\">DY (mm)</td>"
          "<td bgcolor=\"#E0E0E0\">DZ (mm)</td>"
          "<td bgcolor=\"#E0E0E0\">RX (deg)</td>"
          "<td bgcolor=\"#E0E0E0\">RY (deg)</td>"
          "<td bgcolor=\"#E0E0E0\">RZ (deg)</td></tr>\n";

    for (size_t i = 0; i < nTotalNodes; i++)
    {
        ss << "<tr><td>" << problem_to_solve_->getNodeLabel(i) << "</td>";
        for (int k = 0; k < 6; k++)
            if (U[i][k] == 0)
                ss << "<td>0</td>";
            else
            {
                if (k < 3)
                    ss << format("<td>%.03f</td>", U[i][k] * 1e3);
                else
                    ss << format("<td>%.02f</td>", U[i][k] * 180 / M_PI);
            }
        ss << "</tr>\n";
    }
    ss << "</table>\n";
    return ss.str();
}
