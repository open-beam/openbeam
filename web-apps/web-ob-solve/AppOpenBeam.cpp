

#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
// #include "/home/jlblanco/code/mrpt/3rdparty/nanogui/ext/glfw/deps/linmath.h"
#include <GLFW/glfw3.h>
//
#include <localization.h>  // Internationalization support
#include <mrpt/containers/yaml.h>
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
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(
            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        auto& cam = theScene_->getViewport()->getCamera();
        cam.setOrthogonal();
        cam.setElevationDegrees(90);
        cam.setAzimuthDegrees(-90);

        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        const int    viewWidth = vp[2], viewHeight = vp[3];
        const double viewAspectRatio =
            static_cast<double>(viewWidth) / viewHeight;

        // get deformed bbox:
        {
            // Auto determine maximum deformation:
            double MAX_DEFORMATION_SCALE = 0;
            {
                const num_t MAX_DISPL =
                    problem_to_solve_->getMaximumDeformedDisplacement(sInfo_);
                num_t min_x, max_x, min_y, max_y;
                problem_to_solve_->getBoundingBox(min_x, max_x, min_y, max_y);
                const num_t Ax                  = max_x - min_x;
                const num_t Ay                  = max_y - min_y;
                const num_t MAX_ABS_DEFORMATION = std::max(Ax, Ay) * 0.05;
                MAX_DEFORMATION_SCALE = MAX_ABS_DEFORMATION / MAX_DISPL;
            }

            double deformed_scale_factor_for_bbox = MAX_DEFORMATION_SCALE;

            num_t min_x, max_x, min_y, max_y;
            structure_.getBoundingBox(
                min_x, max_x, min_y, max_y, true /*deformed*/, &sInfo_,
                deformed_scale_factor_for_bbox);

            const double Ax          = std::max<double>(max_x - min_x, 1.0);
            const double Ay          = std::max<double>(max_y - min_y, 1.0);
            const double marginRatio = 0.15;
            min_x -= marginRatio * Ax;
            max_x += marginRatio * Ax;
            min_y -= marginRatio * Ay;
            max_y += marginRatio * Ay;

            double verticalFov = 2 * Ay * (1.0 + marginRatio + 0.20 + 0.40);

            // zoom out if horizontal width is too large:
            {
                double horizontalFov = 2 * Ax * (1.0 + marginRatio + 0.20);
                double hzPredFov     = verticalFov * viewAspectRatio;
                if (horizontalFov > hzPredFov)
                    verticalFov *= horizontalFov / hzPredFov;
            }

            cam.setPointingAt((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, 0);
            cam.setZoomDistance(verticalFov);
        }

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

    try
    {
        builtOk_ = false;

        // openbeam::setVerbosityLevel(arg_verbose_level.getValue());

        // Load from file:
        openbeam::vector_string_t errMsg, warnMsg;

        std::stringstream ss(def);
        builtOk_ = structure_.loadFromStream(ss, errMsg, warnMsg);

        // Return errors:
        for (const auto& m : errMsg)
        {
            retStr += "ERR: ";
            retStr += m;
            retStr += "\n";
            builtOk_ = false;
        }
        for (const auto& m : warnMsg)
        {
            retStr += "WARN: ";
            retStr += m;
            retStr += "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        retStr += "ERR: ";
        retStr += e.what();
        builtOk_ = false;
    }

    return retStr;
}

std::string AppOpenBeam::Solve(const std::string& options)
{
    using namespace openbeam;
    using namespace std::string_literals;

    std::string ret;

    try
    {
        const auto o = mrpt::containers::yaml::FromText(options);

        MeshParams mesh_params;
        mesh_params.max_element_length =
            o.getOrDefault<double>("mesh_max_length", 0.10);

        bool doMesh = o.getOrDefault<bool>("mesh", true);

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

        problem_to_solve_->postProcCalcStress(stressInfo_, sInfo_);

        // Stats:
        const size_t nF   = info.free_dof_indices.size();
        const size_t nB   = info.bounded_dof_indices.size();
        const size_t nTot = nF + nB;

        const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();
        ASSERT_(nTot == dofs.size());

        ret += "Nodes: before meshing="s +
               std::to_string(mesh_out_info_.num_original_nodes) + "\n"s;
        ret += problem_to_solve_->getProblemDoFsDescription();
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

    std::stringstream ss;

    try
    {
        BuildProblemInfo&           info = sInfo_.build_info;
        const size_t                nF   = info.free_dof_indices.size();
        const size_t                nB   = info.bounded_dof_indices.size();
        const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();

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
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        ss << e.what();
    }

    return ss.str();
}

std::string AppOpenBeam::GetDisplacementsAsHTML()
{
    using namespace openbeam;
    using namespace openbeam::localization;

    std::stringstream ss;

    try
    {
        BuildProblemInfo&           info = sInfo_.build_info;
        const size_t                nF   = info.free_dof_indices.size();
        const size_t                nB   = info.bounded_dof_indices.size();
        const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();
        const size_t nTotalNodes = problem_to_solve_->getNumberOfNodes();

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

        ss << "<h3>" << _t(STR_Displacements) << " (U<sub>R</sub> "
           << _t(STR_and) << " U<sub>L</sub>):</h3>\n";

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
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        ss << e.what();
    }
    return ss.str();
}

std::string AppOpenBeam::GetStressAsHTML()
{
    using namespace openbeam;
    using namespace openbeam::localization;

    std::stringstream ss;

    try
    {
        BuildProblemInfo&           info = sInfo_.build_info;
        const size_t                nF   = info.free_dof_indices.size();
        const size_t                nB   = info.bounded_dof_indices.size();
        const std::vector<NodeDoF>& dofs = problem_to_solve_->getProblemDoFs();
        const size_t nTotalNodes = problem_to_solve_->getNumberOfNodes();

        ss << "<h4>Element stress:</h4>\n";

        ss << "<table border=\"1\" cellspacing=\"0\" cellpadding=\"3\">\n"
              "<tr><td>Element</td><td>Face</td><td>N</td><td>Vy</"
              "td><td>Vz</td><td>Mx</td><td>My</td><td>Mz</td></tr>\n";

        const unsigned int nE =
            static_cast<unsigned int>(stressInfo_.element_stress.size());
        for (unsigned int i = 0; i < nE; i++)
        {
            for (unsigned int face = 0;
                 face < stressInfo_.element_stress[i].size(); face++)
            {
                const FaceStress& es = stressInfo_.element_stress[i][face];
                ss << format(
                    "<tr><td align=\"center\">%s</td><td "
                    "align=\"center\">%2u (%s)</td>",
                    face == 0 ? format(" %5u ", i).c_str() : "       ", face,
                    problem_to_solve_
                        ->getNodeLabel(problem_to_solve_->getElement(i)
                                           ->conected_nodes_ids[face])
                        .c_str());

                const num_t nums[6] = {es.N, es.Vy, es.Vz, es.Mx, es.My, es.Mz};

                for (int i = 0; i < 6; i++)
                {
                    ss << "<td align=\"right\">";
                    if (nums[i] == 0)
                        ss << "0";
                    else
                        ss << format("%f", nums[i]);
                    ss << "</td>";
                }
                ss << "</tr>\n";
            }
        }

        ss << "</table>\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        ss << e.what();
    }

    return ss.str();
}

void AppOpenBeam::generateVisualization(const std::string& options)
{
    if (!builtOk_) return;

    try
    {
        openbeam::DrawStructureOptions draw_options;
        draw_options.show_nodes_original    = true;
        draw_options.show_nodes_deformed    = false;
        draw_options.show_elements_original = true;
        draw_options.show_elements_deformed = false;

        // Auto-scale with structure dimensions:
        num_t min_x, max_x, min_y, max_y;
        structure_.getBoundingBox(
            min_x, max_x, min_y, max_y, false /*deformed*/, &sInfo_, 0);
        const num_t Ax       = max_x - min_x;
        const num_t Ay       = max_y - min_y;
        const num_t MAX_DIMS = std::max(Ax, Ay);

        draw_options.NODE_RADIUS        = MAX_DIMS * 0.0075;
        draw_options.BEAM_PINNED_RADIUS = 1.5 * draw_options.NODE_RADIUS;
        draw_options.EDGE_WIDTH         = 1.2 * draw_options.NODE_RADIUS;

        const auto o = mrpt::containers::yaml::FromText(options);
        draw_options.loadFromYaml(o);

        auto glObj = problem_to_solve_->getVisualization(
            draw_options, sInfo_, mesh_info_, &stressInfo_);

        theScene_->clear();
        theScene_->insert(glObj);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
    }
}

double AppOpenBeam::determineAutoDeformationScale()
{
    if (!problem_to_solve_) return 1.0;

    const num_t MAX_DISPL =
        problem_to_solve_->getMaximumDeformedDisplacement(sInfo_);

    num_t min_x, max_x, min_y, max_y;
    problem_to_solve_->getBoundingBox(min_x, max_x, min_y, max_y);
    const num_t Ax                    = max_x - min_x;
    const num_t Ay                    = max_y - min_y;
    const num_t MAX_ABS_DEFORMATION   = std::max(Ax, Ay) * 0.05;
    double      MAX_DEFORMATION_SCALE = MAX_ABS_DEFORMATION / MAX_DISPL;

    return MAX_DEFORMATION_SCALE;
}
