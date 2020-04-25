/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2013  Jose Luis Blanco Claraco                       |
   |                              University of Malaga                         |
   |                                                                           |
   | OpenBeam is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   | OpenBeam is distributed in the hope that it will be useful,               |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with OpenBeam.  If not, see <http://www.gnu.org/licenses/>.     |
   |                                                                           |
   +---------------------------------------------------------------------------+
 */

#include <localization.h>  // Internationalization support
#include <openbeam/openbeam.h>
#include <tclap/CmdLine.h>

#include <stdio.h>  // for unlink()
#include <iostream>
#include <memory>  // for auto_ptr<>

#ifdef __GNUC__
#include <sys/stat.h>  // mkdir()
#include <unistd.h>
#endif

#ifdef _MSC_VER
#include <direct.h>  // mkdir()
#define mkdir _mkdir
#define chdir _chdir
#endif

using namespace openbeam;
using namespace openbeam::localization;
using namespace std;

// Auxiliary functions (Implemented below) ----------------------------------
template <class MATRIX, class ARRAY>
void print_html_matrix(
    const MATRIX& K, const vector<string>& row_titles,
    const vector<string>& col_titles, const ARRAY* rows_to_use = NULL,
    const ARRAY* cols_to_use = NULL);
template <class MATRIX, class ARRAY>
void print_csv_matrix(
    const std::string& out_csv_file, const MATRIX& K,
    const vector<string>& row_titles, const vector<string>& col_titles,
    const ARRAY* rows_to_use = NULL, const ARRAY* cols_to_use = NULL);

void usedDOFs2titles(
    size_t node_id, vector<string>& titles, bool html,
    const TUsedDoFs* edge_dofs = NULL);
void listDOFs2titles(
    const std::vector<TDoF>& dofs, vector<string>& dof_titles, bool html);

// --------------------------------------------------------------------------

struct TGraphData
{
    std::string    filePrefix;
    std::string    title;
    std::string    x_label, y_label;  // Plot axis labels
    vector<double> x, y;  // Plot data
};

int main_code(int argc, char** argv)
{
    // -------------------------------------------------------------
    // Process command line args:
    // ----------------------------------------------------------
    // Declare supported options.
    TCLAP::CmdLine cmd("ob-solve", ' ', "" /* version */);

    TCLAP::ValueArg<std::string> arg_problemFile(
        "i", "input", "Problem file to load", true, "", "problem.txt", cmd);
    TCLAP::ValueArg<int> arg_verbose_level(
        "v", "verbose", "Verbosity level (0-3)", false, 1, "0-3", cmd);
    TCLAP::ValueArg<std::string> arg_language(
        "l", "lang", "Language (ISO 639-1 code)", false, "en", "en,es,...",
        cmd);
    TCLAP::ValueArg<std::string> arg_output(
        "o", "output", "Switch (text or html) output to a target file", false,
        "", "output.html", cmd);

    TCLAP::SwitchArg arg_meshing(
        "", "mesh", "Enable meshing (Default: no)", cmd);
    TCLAP::ValueArg<double> arg_mesh_resolution(
        "", "mesh-resolution", "Meshing resolution (Default: 0.05 m)", false,
        0.05, "0.05", cmd);

    TCLAP::SwitchArg arg_html(
        "", "html", "Enable HTML output (Default: no)", cmd);
    TCLAP::SwitchArg arg_stress(
        "", "stress", "Show element stress as text (Default: no)", cmd);
    TCLAP::SwitchArg arg_stress_plots(
        "", "stress-plots",
        "Show element stress as graphs, if using HTML output (Default: no)",
        cmd);
    TCLAP::SwitchArg arg_svg(
        "", "svg", "Enable generation of SVG images (Default: no)", cmd);
    TCLAP::SwitchArg arg_sep_plots(
        "", "sep-plots", "Generate separate plot files (Default: no)", cmd);

    TCLAP::ValueArg<std::string> arg_plots_continuous_beam(
        "", "plots-continuous-beam",
        "Generate plots for a sequence of elements which are interpreted as "
        "one single continuous beam",
        false, "", "0,1,2", cmd);

    TCLAP::ValueArg<std::string> arg_plots_width(
        "", "plots-width", "Stress and deformation plot sizes for HTML output",
        false, "1000px", "1000px", cmd);
    TCLAP::ValueArg<std::string> arg_plots_height(
        "", "plots-height", "Stress and deformation plot sizes for HTML output",
        false, "300px", "300px", cmd);

    TCLAP::SwitchArg arg_show_FL(
        "", "show-FL",
        "Text display of the vector of forces in free nodes (Default: no)",
        cmd);
    TCLAP::SwitchArg arg_show_FR(
        "", "show-FR", "Text display of reactions (Default: no)", cmd);
    TCLAP::SwitchArg arg_show_U(
        "", "show-U", "Text display of all displecements (Default: no)", cmd);

    TCLAP::SwitchArg arg_full_K(
        "", "full-k",
        "Enable displaying the full stiffness matrix (K) for the entire "
        "problem (Default: no)",
        cmd);
    TCLAP::ValueArg<std::string> arg_full_K_csv(
        "", "full-k-csv", "Dumps full K matrix to a CSV file", false, "",
        "full-k.csv", cmd);
    TCLAP::SwitchArg arg_all_elements_K(
        "", "all-element-k",
        "Enable displaying the global stiffness matrices for every element "
        "(Default: no)",
        cmd);

    TCLAP::ValueArg<std::string> arg_out_animation(
        "", "animate-deformed",
        "Creates an animation of the structure deformed state and saves to "
        "target GIF file",
        false, "", "anim-deformed.gif", cmd);
    TCLAP::ValueArg<double> arg_out_images_width(
        "", "out-images-width", "Width of SVG images & animations (in pixels)",
        false, 512, "512", cmd);
    TCLAP::ValueArg<unsigned int> arg_out_animation_num_frames(
        "", "out-animation-num-frames",
        "When generation animations, the number of keyframes", false, 30, "30",
        cmd);
    TCLAP::ValueArg<unsigned int> arg_out_animation_frame_delay(
        "", "out-animation-frame-delay",
        "When generation animations, the delay (ms) between keyframes", false,
        10, "10", cmd);
    TCLAP::SwitchArg arg_anim_show_element_labels(
        "", "anim-show-element-labels",
        "Show labels in animation (Default: no)", cmd);
    TCLAP::SwitchArg arg_anim_show_node_labels(
        "", "anim-show-node-labels", "Show labels in animation (Default: no)",
        cmd);
    TCLAP::ValueArg<std::string> arg_svg_filename_prefix(
        "", "svg-prefix", "Filename prefix", false, "fem", "fem", cmd);
    TCLAP::ValueArg<double> arg_svg_deformed_factor(
        "", "svg-deformed-factor", "Deformation scale factor (0:guess)", false,
        0, "0", cmd);
    TCLAP::SwitchArg arg_anim_keep_files(
        "", "anim-keep-files", "Keep animation keyframe images", cmd);
    TCLAP::ValueArg<std::string> arg_out_dir(
        "", "out-dir", "Output directory", false, ".", ".", cmd);

    TCLAP::SwitchArg arg_draw_mesh("", "mesh-draw", "Visualize mesh", cmd);

    TCLAP::ValueArg<double> arg_margin_bottom(
        "", "draw-margin-bottom", "(Default:auto)", false, -1.0, "1.0", cmd);
    TCLAP::ValueArg<double> arg_margin_top(
        "", "draw-margin-top", "(Default:auto)", false, -1.0, "1.0", cmd);
    TCLAP::ValueArg<double> arg_margin_left(
        "", "draw-margin-left", "(Default:auto)", false, -1.0, "1.0", cmd);
    TCLAP::ValueArg<double> arg_margin_right(
        "", "draw-margin-right", "(Default:auto)", false, -1.0, "1.0", cmd);

    // Parse arguments:
    if (!cmd.parse(argc, argv)) return -1;

    // Change the language of the interface:
    const std::string lang = arg_language.getValue();
    openbeam::localization::selectLanguage(lang);

    const std::string fil_to_load = arg_problemFile.getValue();

    const bool calc_stress =
        arg_stress.getValue() || arg_stress_plots.getValue();
    const bool out_html         = arg_html.getValue();
    const bool out_html_no_head = false;

    const bool        out_svg                 = arg_svg.getValue();
    const bool        out_full_k              = arg_full_K.getValue();
    const bool        out_full_k_csv          = arg_full_K_csv.isSet();
    const std::string out_full_k_csv_filename = arg_full_K_csv.getValue();
    const bool        out_all_elements_k      = arg_all_elements_K.getValue();

    std::vector<TGraphData> lst_html_graphs;

    // -------------------------------------------------------------
    // Load FEM problem:
    // -------------------------------------------------------------
    openbeam::setVerbosityLevel(arg_verbose_level.getValue());

    CStructureProblem problem;

    // Load from file:
    vector_string errMsg, warnMsg;
    problem.loadFromFile(fil_to_load, &errMsg, &warnMsg);

    if (!errMsg.empty())
    {
        for (size_t i = 0; i < errMsg.size(); i++) cerr << errMsg[i] << endl;
        return -1;
    }

    if (arg_out_dir.isSet())
    {
#ifdef __GNUC__
        ::mkdir(arg_out_dir.getValue().c_str(), 0777);
#else
        ::mkdir(arg_out_dir.getValue().c_str());
#endif
        if (0 != ::chdir(arg_out_dir.getValue().c_str()))
            throw std::runtime_error(
                "Error changing to directory in --out-dir");
    }

    // Redirect output to file?
    auto_ptr<CConsoleRedirector> console_redirect;
    if (arg_output.isSet())
    {
        // By creating this object all will be handled automatically: We'll just
        // need to write to cout as normal.
        const std::string fil_redirect_output = arg_output.getValue();
        console_redirect                      = auto_ptr<CConsoleRedirector>(
            new CConsoleRedirector(fil_redirect_output, false, false));
    }

    // Start HTML preamble:
    if (out_html && !out_html_no_head)
    {
        cout << "<!DOCTYPE html>\n"
                "<html>\n"
                "<head>\n"
                "<title>Results for `"
             << arg_problemFile.getValue()
             << "`</title>\n"
                "<meta http-equiv=\"content-type\" content=\"text/html; "
                "charset=utf-8\"/>\n"
                "<script "
                "src=\"https://ajax.googleapis.com/ajax/libs/jquery/1.12.0/"
                "jquery.min.js\"></script>\n"
                "<script "
                "src=\"http://cdnjs.cloudflare.com/ajax/libs/gsap/1.18.0/"
                "TweenMax.min.js\"></script>\n"
                "<script src=\"http://www.google.com/jsapi\"></script>\n"
                "<script>\n"
                "  google.load('visualization', '1', {packages: "
                "['corechart']});\n"
                "</script>\n"
                // For animation -----------
                "<style>\n"
                ".animatedimage {\n"
                "	position: relative;\n"
                "	display: inline-block;\n"
                "	line-height: 0;\n"
                "	overflow: hidden;\n"
                "}\n"
                ".animatedimage > * {\n"
                "	position: absolute;\n"
                "	display: inline-block;\n"
                "	visibility: hidden;\n"
                "	border: 0;\n"
                "}\n"
                "</style>\n"
                // -----------------
                "<head/>\n"
                "<body>\n";
    }

    // -------------------------------------------------------------
    // Mesh?
    // -------------------------------------------------------------
    CStructureProblem problem_mesh;
    TMeshOutputInfo   mesh_out_info;
    TMeshParams       mesh_params;

    mesh_params.max_element_length = arg_mesh_resolution.getValue();

    CStructureProblem* problem_to_solve = NULL;
    TMeshOutputInfo*   mesh_info        = NULL;

    if (arg_meshing.isSet())
    {
        // Mesh:
        problem.mesh(problem_mesh, mesh_out_info, mesh_params);
        problem_to_solve = &problem_mesh;
        mesh_info        = &mesh_out_info;
    }
    else
    {
        // Don't mesh:
        problem_to_solve = &problem;
    }

    // -------------------------------------------------------------
    // Solve:
    // -------------------------------------------------------------
    TStaticSolveProblemInfo sInfo;
    problem_to_solve->solveStatic(sInfo);

    TBuildProblemInfo& info = sInfo.build_info;

    problem_to_solve->assembleProblem(info);

    // Stats:
    const size_t nF   = info.free_dof_indices.size();
    const size_t nB   = info.bounded_dof_indices.size();
    const size_t nTot = nF + nB;

    const std::vector<TDoF>& dofs = problem_to_solve->getProblemDoFs();
    OBASSERT(nTot == dofs.size())

    // Generate animation of the deformed state?
    std::vector<std::string> anim_svg_files;
    if (arg_out_animation.isSet() || arg_anim_keep_files.isSet())
    {
        const string       sOutGIF_filename = arg_out_animation.getValue();
        const unsigned int NUM_FRAMES = arg_out_animation_num_frames.getValue();
        const unsigned int GIF_DELAY = arg_out_animation_frame_delay.getValue();

        // Auto determine maximum deformation:
        num_t MAX_DEFORMATION_SCALE = arg_svg_deformed_factor.getValue();
        if (MAX_DEFORMATION_SCALE == 0.0)
        {
            const num_t MAX_DISPL =
                problem_to_solve->getMaximumDeformedDisplacement(sInfo);
            num_t min_x, max_x, min_y, max_y;
            problem_to_solve->getBoundingBox(min_x, max_x, min_y, max_y);
            const num_t Ax                  = max_x - min_x;
            const num_t Ay                  = max_y - min_y;
            const num_t MAX_ABS_DEFORMATION = std::max(Ax, Ay) * 0.05;
            MAX_DEFORMATION_SCALE           = MAX_ABS_DEFORMATION / MAX_DISPL;
        }

        // Do animation ------------
        openbeam::TDrawStructureOptions draw_options;
        draw_options.image_width = arg_out_images_width.getValue();

        draw_options.show_nodes_original  = true;
        draw_options.show_nodes_deformed  = true;
        draw_options.nodes_original_alpha = 0.15;
        draw_options.constraints_original_alpha =
            draw_options.nodes_original_alpha;

        draw_options.margin_bottom = arg_margin_bottom.getValue();
        draw_options.margin_top    = arg_margin_top.getValue();
        draw_options.margin_left   = arg_margin_left.getValue();
        draw_options.margin_right  = arg_margin_right.getValue();

        draw_options.show_elements_original  = true;
        draw_options.show_elements_deformed  = true;
        draw_options.elements_original_alpha = 0.12;
        draw_options.show_element_labels = arg_anim_show_element_labels.isSet();
        draw_options.show_node_labels    = arg_anim_show_node_labels.isSet();

        draw_options.show_loads = true;

        num_t scale =
            1e-12;  // Can't be exactly zero, since that means autoscale!
        num_t Ascale = MAX_DEFORMATION_SCALE / NUM_FRAMES;
        OB_MESSAGE(2) << "Generating animation frames...\n";

        std::vector<std::string> files_to_delete;
        files_to_delete.reserve(2 * NUM_FRAMES);

        for (unsigned int i = 0; i < 2 * NUM_FRAMES; i++, scale += Ascale)
        {
            draw_options.deformed_scale_factor          = scale;
            draw_options.deformed_scale_factor_for_bbox = MAX_DEFORMATION_SCALE;

            if (arg_anim_keep_files.isSet())
            {
                const string sFil = openbeam::format(
                    "%s_animated_%06u.svg",
                    arg_svg_filename_prefix.getValue().c_str(), i);
                problem_to_solve->saveAsImageSVG(
                    sFil, draw_options, &sInfo,
                    !arg_draw_mesh.isSet() ? mesh_info : NULL);
                anim_svg_files.push_back(sFil);
            }

            if (arg_out_animation.isSet())
            {
                const string sFil = openbeam::format(
                    "%s_animated_%06u.png",
                    arg_svg_filename_prefix.getValue().c_str(), i);
                problem_to_solve->saveAsImagePNG(
                    sFil, draw_options, &sInfo,
                    !arg_draw_mesh.isSet() ? mesh_info : NULL);
                files_to_delete.push_back(sFil);
            }
            if (i == NUM_FRAMES) Ascale = -Ascale;
        }

        if (arg_out_animation.isSet())
        {
            OB_MESSAGE(2) << "Animation frames done!\n";
            const string sCmd = openbeam::format(
                "convert -delay %u -loop 0 %s_animated_*.png %s", GIF_DELAY,
                arg_svg_filename_prefix.getValue().c_str(),
                sOutGIF_filename.c_str());
            OB_MESSAGE(2) << "Animation compile GIF:\n";
            OB_MESSAGE(2) << "[EXTERNAL CMD] " << sCmd << endl;

            const int ret = ::system(sCmd.c_str());
            if (ret)
                cerr << "**ERROR**: Invoking convert command (ret=" << ret
                     << "):\n"
                     << sCmd << endl;

            // Remove temporary images:
            for (size_t i = 0; i < files_to_delete.size(); i++)
                ::unlink(files_to_delete[i].c_str());

            OB_MESSAGE(2) << "Animation done!\n";
        }
    }  // end of "arg_out_animation"

    std::string scripts_before_body_end;

    // Images:
    if (out_html && out_svg)
    {
#if OPENBEAM_HAS_CAIRO
        openbeam::TDrawStructureOptions draw_options;
        draw_options.image_width            = arg_out_images_width.getValue();
        draw_options.show_nodes_original    = true;
        draw_options.show_nodes_deformed    = false;
        draw_options.show_node_labels       = true;
        draw_options.show_element_labels    = true;
        draw_options.show_elements_original = true;
        draw_options.show_elements_deformed = false;

        draw_options.margin_bottom = arg_margin_bottom.getValue();
        draw_options.margin_top    = arg_margin_top.getValue();
        draw_options.margin_left   = arg_margin_left.getValue();
        draw_options.margin_right  = arg_margin_right.getValue();

        const string sFilOriginal =
            arg_svg_filename_prefix.getValue() + string("_original.svg");
        problem_to_solve->saveAsImageSVG(
            sFilOriginal, draw_options, &sInfo,
            !arg_draw_mesh.isSet() ? mesh_info : NULL);

        draw_options.show_nodes_original     = true;
        draw_options.nodes_original_alpha    = 0.2;
        draw_options.show_nodes_deformed     = true;
        draw_options.show_elements_original  = true;
        draw_options.elements_original_alpha = 0.2;
        draw_options.show_elements_deformed  = true;
        draw_options.show_element_labels     = false;
        draw_options.deformed_scale_factor = arg_svg_deformed_factor.getValue();

        const string sFilDeformed =
            arg_svg_filename_prefix.getValue() + string("_deformed.svg");
        TImageSaveOutputInfo img_out_info;
        problem_to_solve->saveAsImageSVG(
            sFilDeformed, draw_options, &sInfo,
            !arg_draw_mesh.isSet() ? mesh_info : NULL, &img_out_info);

        cout << "<div width=\"100%\"> <!-- FIGURES -->\n"
                "<div align=\"center\"><h3>Figures</h3></div>\n";

        if (out_html && !anim_svg_files.empty())
        {
            cout << "<div align=\"center\" >"
                    "<b>Deformation animation:</b> (&times; "
                 << arg_svg_deformed_factor.getValue()
                 << ")<br>\n"
                    " <div class='animatedimage'>\n";
            for (size_t i = 0; i < anim_svg_files.size(); i++)
                cout << " <img src='" << anim_svg_files[i] << "' width=\""
                     << img_out_info.img_width << "\" height=\""
                     << img_out_info.img_height << "\"/>";
            cout << "\n </div>\n"
                    "</div><br/>\n";

            scripts_before_body_end +=
                "<script>\n"
                "var images = $('.animatedimage').children(), // images in the "
                "sequence\n"
                "	fps = 10,\n"
                "	duration = 1 / fps;\n"
                "var sequence = new TimelineMax({ repeat: -1 })\n"
                "	.staggerTo(images, 0, { position: 'static', visibility: "
                "'visible' }, duration, 0)\n"
                "	.staggerTo(images.not(images.last()), 0, { position: "
                "'absolute', visibility: 'hidden', immediateRender: false }, "
                "duration, duration)\n"
                "	.set({}, {}, \"+=\"+duration);\n"
                "</script>\n";
        }

        cout << "<div align=\"center\" >"
                "<b>Original (unloaded) state:</b><br>\n"
                "<a href=\""
             << sFilOriginal
             << "\" target=\"_blank\">"
                "<img src=\""
             << sFilOriginal << "\" width=\"" << arg_out_images_width.getValue()
             << "\"/><br/>\n"
             << "(" << _t(STR_click_to_enlarge)
             << ")</a>"
                "</div><br/>\n";

        cout << "<div align=\"center\" >"
                "<b>Final (deformed) state:</b> (&times; "
             << arg_svg_deformed_factor.getValue()
             << ")<br>\n"
                "<a href=\""
             << sFilDeformed
             << "\" target=\"_blank\">"
                "<img src=\""
             << sFilDeformed << "\" width=\"" << arg_out_images_width.getValue()
             << "\"/><br/>\n"
             << "(" << _t(STR_click_to_enlarge)
             << ")</a>"
                "</div><br/>\n";

        cout << "</div> <!-- END OF FIGURES -->\n";
#endif
    }

    if (!warnMsg.empty())
    {
        if (out_html)
        {
            cout << "<b>Warnings:</b><ul>";
            for (size_t i = 0; i < warnMsg.size(); i++)
                cout << "<li>" << warnMsg[i] << "</li>\n";
        }
        else
        {
            for (size_t i = 0; i < warnMsg.size(); i++)
                cout << warnMsg[i] << endl;
        }
    }

    if (out_html)
        cout << "<div style=\"clear:both;\"> <!-- NON GRAPHICAL RESULTS -->\n";

    // Show FULL Stiffness matrix:
    if (out_full_k || out_full_k_csv)
    {
        TDynMatrix K_full;

        const TDynMatrix Kff = TDynMatrix(info.K_ff);
        const TDynMatrix Kbb = TDynMatrix(info.K_bb);
        const TDynMatrix Kbf = TDynMatrix(info.K_bf);

        K_full.setZero(nTot, nTot);
        for (size_t i1 = 0; i1 < nF; i1++)
        {
            const size_t ii1 = info.free_dof_indices[i1];
            for (size_t i2 = i1; i2 < nF; i2++)
            {
                const size_t ii2 = info.free_dof_indices[i2];
                K_full(ii1, ii2) = K_full(ii2, ii1) = Kff(i1, i2);
            }
        }

        for (size_t i1 = 0; i1 < nB; i1++)
        {
            const size_t ii1 = info.bounded_dof_indices[i1];
            for (size_t i2 = i1; i2 < nB; i2++)
            {
                const size_t ii2 = info.bounded_dof_indices[i2];
                K_full(ii1, ii2) = K_full(ii2, ii1) = Kbb(i1, i2);
            }
        }

        for (size_t i1 = 0; i1 < nB; i1++)
        {
            const size_t ii1 = info.bounded_dof_indices[i1];
            for (size_t i2 = 0; i2 < nF; i2++)
            {
                const size_t ii2 = info.free_dof_indices[i2];
                K_full(ii1, ii2) = K_full(ii2, ii1) = Kbf(i1, i2);
            }
        }

        const TDynMatrix K_full_dense(K_full);

        if (out_full_k)
        {
            if (!out_html)
            {
                cout << _t(STR_GlobalStiffnessMatrix) << "(K):\n";
                cout << "K:\n" << K_full_dense << endl << endl;
            }
            else
            {
                // Build texts of each "title" row/column:
                vector<string> dof_titles;
                listDOFs2titles(dofs, dof_titles, true /*html*/);

                cout << "<h3>" << _t(STR_GlobalStiffnessMatrix)
                     << "(K):</h3>\n";
                print_html_matrix<TDynMatrix, TUsedDoFs>(
                    K_full_dense, dof_titles, dof_titles);
            }
        }

        if (out_full_k_csv)
        {
            // Build texts of each "title" row/column:
            vector<string> dof_titles;
            listDOFs2titles(dofs, dof_titles, false /*html*/);

            print_csv_matrix<TDynMatrix, TUsedDoFs>(
                out_full_k_csv_filename, K_full_dense, dof_titles, dof_titles);
        }

    }  // end full K

    if (out_all_elements_k)
    {
        const size_t nEle = problem_to_solve->getNumberOfElements();

        if (out_html)
            cout << "<h3>" << _t(STR_AllElementsStiffnessMatrices)
                 << "(K):</h3>\n";
        else
            cout << _t(STR_AllElementsStiffnessMatrices) << ":\n";

        for (size_t i = 0; i < nEle; i++)
        {
            if (out_html)
                cout << "<h4>"
                     << "E" << i << ":</h4>\n";
            else
                cout << "E" << i << ":\n";

            const CElement* el = problem_to_solve->getElement(i);
            openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t subMats;
            el->getGlobalStiffnessMatrices(subMats);

            vector<TUsedDoFs> edge_dofs;  // In each edge
            el->getGlobalDoFs(edge_dofs);

            for (size_t i = 0; i < subMats.size(); i++)
            {
                const TStiffnessSubmatrix& sm = subMats[i];
                const size_t node_in  = el->conected_nodes_ids[sm.edge_in];
                const size_t node_out = el->conected_nodes_ids[sm.edge_out];

                const TUsedDoFs& edge_dofs_in  = edge_dofs[sm.edge_in];
                const TUsedDoFs& edge_dofs_out = edge_dofs[sm.edge_out];

                vector<string> col_titles, row_titles;

                usedDOFs2titles(
                    node_in, col_titles,
                    out_html);  // generate all captions, they'll be
                usedDOFs2titles(
                    node_out, row_titles,
                    out_html);  // filtered in print_html_matrix()

                if (out_html)
                {
                    cout << format(
                        "K<sub>%u,%u</sub> = ",
                        static_cast<unsigned int>(node_in),
                        static_cast<unsigned int>(node_out));
                    print_html_matrix<TMatrix66, TUsedDoFs>(
                        sm.matrix, row_titles, col_titles, &edge_dofs_out,
                        &edge_dofs_in);
                }
                else
                    cout << sm.matrix << endl << endl;
            }
        }

    }  // end out_all_elements_k

    if (out_html)
    {
        if (arg_show_FL.isSet())
        {
            cout << "<h3>" << _t(STR_Loads) << "(F<sub>L</sub>):</h3>\n";

            cout
                << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";
            cout << "<tr><td bgcolor=\"#E0E0E0\">" << _t(STR_dof)
                 << "</td> <td bgcolor=\"#E0E0E0\">(N, Nm)</td></tr>\n";

            for (size_t i = 0; i < nF; i++)
            {
                const TDoF& dof = dofs[info.free_dof_indices[i]];
                cout << "<tr><td>";
                switch (dof.dof)
                {
                    case 0:
                        cout << "F<sub>x";
                        break;
                    case 1:
                        cout << "F<sub>y";
                        break;
                    case 2:
                        cout << "F<sub>z";
                        break;
                    case 3:
                        cout << "M<sub>x";
                        break;
                    case 4:
                        cout << "M<sub>y";
                        break;
                    case 5:
                        cout << "M<sub>z";
                        break;
                };
                cout << dof.node_id;
                cout << "</sub>";
                cout << "</td><td>";
                cout << (info.F_f[i] == 0.0) ? std::string("0")
                                             : format("%.2f", info.F_f[i]);
                cout << "</td></tr>\n";
            }

            cout << "</table>\n";
            cout << "<p>&nbsp;</p>";
        }

        if (arg_show_FR.isSet())
        {
            cout << "<h3>"
                 << "(F<sub>R</sub>):</h3>\n";

            cout
                << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";
            cout << "<tr><td bgcolor=\"#E0E0E0\">" << _t(STR_dof)
                 << "</td> <td bgcolor=\"#E0E0E0\">(N, Nm)</td></tr>\n";

            for (size_t i = 0; i < nB; i++)
            {
                const TDoF& dof = dofs[info.bounded_dof_indices[i]];
                cout << "<tr><td>";
                switch (dof.dof)
                {
                    case 0:
                        cout << "F<sub>x";
                        break;
                    case 1:
                        cout << "F<sub>y";
                        break;
                    case 2:
                        cout << "F<sub>z";
                        break;
                    case 3:
                        cout << "M<sub>x";
                        break;
                    case 4:
                        cout << "M<sub>y";
                        break;
                    case 5:
                        cout << "M<sub>z";
                        break;
                };
                cout << dof.node_id;
                cout << "</sub>";
                cout << "</td><td>";
                cout << format("%.2f", sInfo.F_b[i]);
                cout << "</td></tr>\n";
            }

            cout << "</table>\n";
            cout << "<p>&nbsp;</p>";
        }
    }

    // Build list of ALL displacements:
    // -------------------------------------
    const size_t           nTotalNodes = problem_to_solve->getNumberOfNodes();
    vector<vector<double>> U(nTotalNodes);
    for (size_t i = 0; i < nTotalNodes; i++) U[i].assign(6, 0);

    for (size_t i = 0; i < nF; i++)
    {
        const TDoF& dof         = dofs[info.free_dof_indices[i]];
        U[dof.node_id][dof.dof] = sInfo.U_f[i];
    }
    for (size_t i = 0; i < nB; i++)
    {
        const TDoF& dof         = dofs[info.bounded_dof_indices[i]];
        U[dof.node_id][dof.dof] = info.U_b[i];
    }

    if (arg_show_U.isSet())
    {
        if (out_html)
        {
            cout << "<h3>" << _t(STR_Displacements) << " (U<sub>R</sub> "
                 << _t(STR_and) << " U<sub>L</sub>):</h3>\n";

            cout
                << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";

            cout << "<tr>"
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
                cout << "<tr><td>" << problem_to_solve->getNodeLabel(i)
                     << "</td>";
                for (int k = 0; k < 6; k++)
                    if (U[i][k] == 0)
                        cout << "<td>0</td>";
                    else
                    {
                        if (k < 3)
                            cout << format("<td>%.03f</td>", U[i][k] * 1e3);
                        else
                            cout << format(
                                "<td>%.02f</td>", U[i][k] * 180 / M_PI);
                    }
                cout << "</tr>\n";
            }
            cout << "</table>\n";
        }
        else
        {
            cout << "\n\nRESULTS:\n------------------------\n";
            cout << _t(STR_Displacements) << " (U_f):\n" << sInfo.U_f << endl;
            cout << _t(STR_Reactions) << " (F_b):\n"
                 << sInfo.F_b << endl
                 << endl;
        }
    }

    //	if (out_html) cout << "</pre>\n";

    CStructureProblem::TStressInfo stressInfo;
    if (calc_stress)
    { problem_to_solve->postProcCalcStress(stressInfo, sInfo); }

    // Stress: textual output:
    // ------------------------------
    if (arg_stress.isSet())
    {
        if (!out_html)
        {
            cout << "\n\nELEMENT STRESS:\n\n";
            cout << "Element|Face|     N    |     Vy   |     Vz   |     Mx   | "
                    "    My   |     Mz   |\n";
            cout << "-------+----+----------+----------+----------+----------+-"
                    "---------+----------+\n";
        }
        else
        {
            cout << "<br /><br />\n"
                    "<h4>Element stress:</h4>\n";

            cout << "<table border=\"1\" cellspacing=\"0\" cellpadding=\"3\">\n"
                    "<tr><td>Element</td><td>Face</td><td>N</td><td>Vy</"
                    "td><td>Vz</td><td>Mx</td><td>My</td><td>Mz</td></tr>\n";
        }

        const unsigned int nE =
            static_cast<unsigned int>(stressInfo.element_stress.size());
        for (unsigned int i = 0; i < nE; i++)
        {
            for (unsigned int face = 0;
                 face < stressInfo.element_stress[i].size(); face++)
            {
                const TFaceStress& es = stressInfo.element_stress[i][face];
                if (!out_html)
                {
                    cout << format(
                        "%s| %2u |%10.3e|%10.3e|%10.3e|%10.3e|%10.3e|%10.3e|\n",
                        face == 0 ? format(" %5u ", i).c_str() : "       ",
                        face, es.N, es.Vy, es.Vz, es.Mx, es.My, es.Mz);
                }
                else
                {
                    cout << format(
                        "<tr><td align=\"center\">%s</td><td "
                        "align=\"center\">%2u (%s)</td>",
                        face == 0 ? format(" %5u ", i).c_str() : "       ",
                        face,
                        problem_to_solve
                            ->getNodeLabel(problem_to_solve->getElement(i)
                                               ->conected_nodes_ids[face])
                            .c_str());

                    const num_t nums[6] = {es.N,  es.Vy, es.Vz,
                                           es.Mx, es.My, es.Mz};

                    for (int i = 0; i < 6; i++)
                    {
                        cout << "<td align=\"right\">";
                        if (nums[i] == 0)
                            cout << "0";
                        else
                            cout << format("%f", nums[i]);
                        cout << "</td>";
                    }
                    cout << "</tr>\n";
                }
            }
        }

        if (out_html) { cout << "</table>\n"; }
    }

    // Create HTML graphs of stress for each meshed element:
    // -----------------------------------------------------
    if (arg_stress_plots.isSet() && (out_html || arg_sep_plots.isSet()) &&
        mesh_info != NULL)
    {
        const TMeshOutputInfo& mi = *mesh_info;

        // Plot sizes:
        const string sGraphStyle =
            "style='width: " + arg_plots_width.getValue() +
            "; height: " + arg_plots_height.getValue() + ";'";

        // For each original bar:
        std::vector<size_t> idx_of_first_plot_for_each_beam;
        std::stringstream   sIndivPlotsBuf;

        lst_html_graphs.reserve(mi.element2elements.size() * 3);
        for (size_t idx_bar = 0; idx_bar < mi.element2elements.size();
             idx_bar++)
        {
            // Find out the original length of the bar:
            const std::vector<size_t>& org_nodes_idxs =
                mi.element2nodes[idx_bar];
            const size_t node0 = *org_nodes_idxs.begin();
            const size_t node1 = *org_nodes_idxs.rbegin();
            const double org_bar_length =
                problem_to_solve->getNodePose(node0).distanceTo(
                    problem_to_solve->getNodePose(node1));

            const std::vector<size_t>& idxs =
                mi.element2elements[idx_bar];  // indices of all the elements
                                               // that build this bar
            const std::vector<size_t>& idxs_nodes =
                mi.element2nodes[idx_bar];  // indices of all the nodes that
                                            // build this bar

            const size_t nE = idxs.size();
            OBASSERT(idxs_nodes.size() == nE + 1)

            // N/Vy/Mz/Mx stress: add 4 graphs
            // Displacement field X,Y,RotZ,RotX: add 4 graphs
            // ---------------------------------------------
            const size_t idx_N     = lst_html_graphs.size();
            const size_t idx_Vy    = idx_N + 1;
            const size_t idx_Mz    = idx_N + 2;
            const size_t idx_Ux    = idx_N + 3;
            const size_t idx_Uy    = idx_N + 4;
            const size_t idx_URotz = idx_N + 5;
            const size_t idx_Mx    = idx_N + 6;
            const size_t idx_URotx = idx_N + 7;

            idx_of_first_plot_for_each_beam.push_back(
                idx_N);  // Associate: idx_bar <==> idx_N

            lst_html_graphs.resize(idx_N + 8);

            const string sFilPrefix =
                openbeam::format("element%03u_", (unsigned int)idx_bar);

            TGraphData& gd_N = lst_html_graphs[idx_N];
            gd_N.filePrefix  = sFilPrefix;
            gd_N.title       = "Axil: N (kN)";
            gd_N.x_label     = "x (m)";
            gd_N.y_label     = "N (kN)";
            gd_N.x.resize(nE + 1);
            gd_N.y.resize(nE + 1);

            TGraphData& gd_Vy = lst_html_graphs[idx_Vy];
            gd_Vy.filePrefix  = sFilPrefix;
            gd_Vy.title       = "Cortante: Vy (kN)";
            gd_Vy.x_label     = "x (m)";
            gd_Vy.y_label     = "Vy (kN)";
            gd_Vy.x.resize(nE + 1);
            gd_Vy.y.resize(nE + 1);

            TGraphData& gd_Mz = lst_html_graphs[idx_Mz];
            gd_Mz.filePrefix  = sFilPrefix;
            gd_Mz.title       = "Flector: Mz (kNm)";
            gd_Mz.x_label     = "x (m)";
            gd_Mz.y_label     = "Mz (kNm)";
            gd_Mz.x.resize(nE + 1);
            gd_Mz.y.resize(nE + 1);

            TGraphData& gd_Ux = lst_html_graphs[idx_Ux];
            gd_Ux.filePrefix  = sFilPrefix;
            gd_Ux.title       = "Desplazamiento: $\\delta_x$ (mm)";
            gd_Ux.x_label     = "x (m)";
            gd_Ux.y_label     = "$\\delta_x$ (mm)";
            gd_Ux.x.resize(nE + 1);
            gd_Ux.y.resize(nE + 1);

            TGraphData& gd_Uy = lst_html_graphs[idx_Uy];
            gd_Uy.filePrefix  = sFilPrefix;
            gd_Uy.title       = "Desplazamiento: $\\delta_y$ (mm)";
            gd_Uy.x_label     = "x (m)";
            gd_Uy.y_label     = "$\\delta_y$ (mm)";
            gd_Uy.x.resize(nE + 1);
            gd_Uy.y.resize(nE + 1);

            TGraphData& gd_Rotz = lst_html_graphs[idx_URotz];
            gd_Rotz.filePrefix  = sFilPrefix;
            gd_Rotz.title       = "Giro Z: $\\theta_z$ ($^\\circ$)";
            gd_Rotz.x_label     = "x (m)";
            gd_Rotz.y_label     = "$\\theta_z$ ($^\\circ$)";
            gd_Rotz.x.resize(nE + 1);
            gd_Rotz.y.resize(nE + 1);

            TGraphData& gd_Mx = lst_html_graphs[idx_Mx];
            gd_Mx.filePrefix  = sFilPrefix;
            gd_Mx.title       = "Torsor: Mx (kNm)";
            gd_Mx.x_label     = "x (m)";
            gd_Mx.y_label     = "Mx (kNm)";
            gd_Mx.x.resize(nE + 1);
            gd_Mx.y.resize(nE + 1);

            TGraphData& gd_Rotx = lst_html_graphs[idx_URotx];
            gd_Rotx.filePrefix  = sFilPrefix;
            gd_Rotx.title       = "Giro X: $\\theta_x$ ($^\\circ$)";
            gd_Rotx.x_label     = "x (m)";
            gd_Rotx.y_label     = "$\\theta_x$ ($^\\circ$)";
            gd_Rotx.x.resize(nE + 1);
            gd_Rotx.y.resize(nE + 1);

            double x  = 0;
            double Al = org_bar_length / nE;
            for (size_t k = 0; k <= nE; k++)
            {
                const size_t idx_el = idxs[k < nE ? k : nE - 1];

                OBASSERT(
                    stressInfo.element_stress[idx_el].size() ==
                    2)  // We only have linear elements only yet!

                unsigned int face = k < nE ? 0 : 1;

                // Stress:
                const TFaceStress& es = stressInfo.element_stress[idx_el][face];
                gd_N.x[k]             = x;
                gd_N.y[k]             = 1e-3 * es.N;
                gd_Vy.x[k]            = x;
                gd_Vy.y[k] =
                    -1e-3 * es.Vy;  // WARNING: The "-" is to match the sign
                                    // criterion as explained in our courses
                gd_Mz.x[k] = x;
                gd_Mz.y[k] = 1e-3 * es.Mz;
                gd_Mx.x[k] = x;
                gd_Mx.y[k] = 1e-3 * es.Mx;

                // Displacement field:
                const size_t idx_node = idxs_nodes[k];

                gd_Ux.x[k]   = x;
                gd_Ux.y[k]   = 1e3 * U[idx_node][0];
                gd_Uy.x[k]   = x;
                gd_Uy.y[k]   = 1e3 * U[idx_node][1];
                gd_Rotz.x[k] = x;
                gd_Rotz.y[k] = (180 / M_PI) * U[idx_node][5];
                gd_Rotx.x[k] = x;
                gd_Rotx.y[k] = (180 / M_PI) * U[idx_node][3];

                x += Al;
            }

            // HTML Place holder for the graph:
            sIndivPlotsBuf << "<h3>" << openbeam::localization::_t(STR_Bar)
                           << " " << idx_bar << "</h3>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_N << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_Vy << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_Mz << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_Ux << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_Uy << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_URotz << "' "
                           << sGraphStyle << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_Mx << "' " << sGraphStyle
                           << "></div>\n";
            sIndivPlotsBuf << "<div id='graph_" << idx_URotx << "' "
                           << sGraphStyle << "></div>\n";
        }

        // Generate extra plots for continuous beams?
        // ----------------------------------------------------
        if (arg_plots_continuous_beam.isSet())
        {
            const string  sContBeamsIDs = arg_plots_continuous_beam.getValue();
            vector_string sLstContBeamIDs;
            tokenize(sContBeamsIDs, ", ", sLstContBeamIDs);

            // N/Vy/Mz stress: add 3 graphs
            // Displacement field X,Y,RotZ: add 3 graphs
            // ---------------------------------------------
            const size_t idx_N     = lst_html_graphs.size();
            const size_t idx_Vy    = idx_N + 1;
            const size_t idx_Mz    = idx_N + 2;
            const size_t idx_Ux    = idx_N + 3;
            const size_t idx_Uy    = idx_N + 4;
            const size_t idx_URotz = idx_N + 5;
            const size_t idx_Mx    = idx_N + 6;
            const size_t idx_URotx = idx_N + 7;

            lst_html_graphs.resize(idx_N + 8);
            const string sFilPrefix =
                openbeam::format("elements_%s_", sContBeamsIDs.c_str());

            TGraphData& gd_N = lst_html_graphs[idx_N];
            gd_N.title       = "Axil: N (kN)";
            gd_N.x_label     = "x (m)";
            gd_N.y_label     = "N (kN)";
            gd_N.filePrefix  = sFilPrefix;

            TGraphData& gd_Vy = lst_html_graphs[idx_Vy];
            gd_Vy.title       = "Cortante: Vy (kN)";
            gd_Vy.x_label     = "x (m)";
            gd_Vy.y_label     = "Vy (kN)";
            gd_Vy.filePrefix  = sFilPrefix;

            TGraphData& gd_Mz = lst_html_graphs[idx_Mz];
            gd_Mz.title       = "Flector: Mz (kNm)";
            gd_Mz.x_label     = "x (m)";
            gd_Mz.y_label     = "Mz (kNm)";
            gd_Mz.filePrefix  = sFilPrefix;

            TGraphData& gd_Ux = lst_html_graphs[idx_Ux];
            gd_Ux.title       = "Desplazamiento $\\delta_x$ (mm)";
            gd_Ux.x_label     = "x (m)";
            gd_Ux.y_label     = "$\\delta_x$ (mm)";
            gd_Ux.filePrefix  = sFilPrefix;

            TGraphData& gd_Uy = lst_html_graphs[idx_Uy];
            gd_Uy.title       = "Desplazamiento $\\delta_y$ (mm)";
            gd_Uy.x_label     = "x (m)";
            gd_Uy.y_label     = "$\\delta_y$ (mm)";
            gd_Uy.filePrefix  = sFilPrefix;

            TGraphData& gd_Rotz = lst_html_graphs[idx_URotz];
            gd_Rotz.title       = "Giro Z: \\u03B8z (\\u00B0)";
            gd_Rotz.x_label     = "x (m)";
            gd_Rotz.y_label     = "\\u03B8z (\\u00B0)";
            gd_Rotz.filePrefix  = sFilPrefix;

            TGraphData& gd_Mx = lst_html_graphs[idx_Mx];
            gd_Mx.title       = "Torsor: Mx (kNm)";
            gd_Mx.x_label     = "x (m)";
            gd_Mx.y_label     = "Mx (kNm)";
            gd_Mx.filePrefix  = sFilPrefix;

            TGraphData& gd_Rotx = lst_html_graphs[idx_URotx];
            gd_Rotx.title       = "Giro X: \\u03B8x (\\u00B0)";
            gd_Rotx.x_label     = "x (m)";
            gd_Rotx.y_label     = "\\u03B8x (\\u00B0)";
            gd_Rotx.filePrefix  = sFilPrefix;

            // Accumulate the graphs from a number of other graphs:
            double x0 = 0;
            for (size_t i = 0; i < sLstContBeamIDs.size(); i++)
            {
                const size_t beam_id = atol(sLstContBeamIDs[i].c_str());
                if (beam_id >= idx_of_first_plot_for_each_beam.size())
                    throw std::runtime_error(format(
                        "Index of continuous beam out of range: '%s'",
                        sContBeamsIDs.c_str()));
                const size_t idx_N_i = idx_of_first_plot_for_each_beam[beam_id];

                const TGraphData& gd_N_i    = lst_html_graphs[idx_N_i];
                const TGraphData& gd_Vy_i   = lst_html_graphs[idx_N_i + 1];
                const TGraphData& gd_Mz_i   = lst_html_graphs[idx_N_i + 2];
                const TGraphData& gd_Ux_i   = lst_html_graphs[idx_N_i + 3];
                const TGraphData& gd_Uy_i   = lst_html_graphs[idx_N_i + 4];
                const TGraphData& gd_Rotz_i = lst_html_graphs[idx_N_i + 5];
                const TGraphData& gd_Mx_i   = lst_html_graphs[idx_N_i + 6];
                const TGraphData& gd_Rotx_i = lst_html_graphs[idx_N_i + 7];

                // Append "x" data:
                for (size_t k = 0; k < gd_N_i.x.size(); k++)
                    gd_N.x.push_back(x0 + gd_N_i.x[k]);
                x0 += *gd_N_i.x.rbegin();

                // Append "y" data:
                gd_N.y.insert(gd_N.y.end(), gd_N_i.y.begin(), gd_N_i.y.end());
                gd_Vy.y.insert(
                    gd_Vy.y.end(), gd_Vy_i.y.begin(), gd_Vy_i.y.end());
                gd_Mz.y.insert(
                    gd_Mz.y.end(), gd_Mz_i.y.begin(), gd_Mz_i.y.end());
                gd_Ux.y.insert(
                    gd_Ux.y.end(), gd_Ux_i.y.begin(), gd_Ux_i.y.end());
                gd_Uy.y.insert(
                    gd_Uy.y.end(), gd_Uy_i.y.begin(), gd_Uy_i.y.end());
                gd_Rotz.y.insert(
                    gd_Rotz.y.end(), gd_Rotz_i.y.begin(), gd_Rotz_i.y.end());
                gd_Mx.y.insert(
                    gd_Mx.y.end(), gd_Mx_i.y.begin(), gd_Mx_i.y.end());
                gd_Rotx.y.insert(
                    gd_Rotx.y.end(), gd_Rotx_i.y.begin(), gd_Rotx_i.y.end());
            }

            gd_Vy.x   = gd_N.x;
            gd_Mz.x   = gd_N.x;
            gd_Ux.x   = gd_N.x;
            gd_Uy.x   = gd_N.x;
            gd_Rotz.x = gd_N.x;
            gd_Mx.x   = gd_N.x;
            gd_Rotx.x = gd_N.x;

            // HTML Place holder for the graph:
            cout << "<h3>" << openbeam::localization::_t(STR_Bar) << " "
                 << sContBeamsIDs << "</h3>\n";
            cout << "<div id='graph_" << idx_N << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_Vy << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_Mz << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_Ux << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_Uy << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_URotz << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_Mx << "' " << sGraphStyle
                 << "></div>\n";
            cout << "<div id='graph_" << idx_URotx << "' " << sGraphStyle
                 << "></div>\n";
        }

        cout << sIndivPlotsBuf.str();
    }

    if (out_html) cout << "</div> <!-- END OF NON GRAPHICAL RESULTS -->\n";

    // Write all HTML graphs:
    if (out_html)
    {
        cout << "<script type=\"text/javascript\">\n"
                "  function drawAllPlots() {\n";

        for (size_t i = 0; i < lst_html_graphs.size(); i++)
        {
            const TGraphData& gd = lst_html_graphs[i];

            cout << "    var sResult_" << i
                 << " = [\n"
                    "['"
                 << gd.x_label << "', '" << gd.y_label << "'],\n";
            for (size_t k = 0; k < gd.x.size(); k++)
                cout << "[" << gd.x[k] << "," << gd.y[k] << "],";
            cout << "];\n";

            cout << "    var data_" << i
                 << " = google.visualization.arrayToDataTable( sResult_" << i
                 << " );\n"
                    "    var props_"
                 << i
                 << " = {legend: { position: 'none'},\n"
                    "		vAxis :  {title: '"
                 << gd.y_label
                 << "', titleTextStyle: { }, gridlines : {count : 5 } },\n"
                    "		hAxis :  {title: '"
                 << gd.x_label
                 << "', titleTextStyle: { }, gridlines : {count : 4 } },\n"
                    "		pointSize : 2,\n"
                    "		title : '"
                 << gd.title
                 << "',\n"
                    "		};\n"
                    "    var obj_Graph_"
                 << i
                 << " = new "
                    "google.visualization.LineChart(document.getElementById('"
                    "graph_"
                 << i << "')).draw(data_" << i << ", props_" << i << " );\n";
        }

        cout << "  }\n"
                "  google.setOnLoadCallback(drawAllPlots);\n"
                "</script>\n";
    }

    // generate individual graph files:
    if (arg_sep_plots.isSet())
    {
        const char* sPythonTemplate =
            "import matplotlib as mpl\n"
            "mpl.use('Agg')\n"
            "import matplotlib.pyplot as plt\n"
            "\n"
            "fig = plt.figure(figsize = (15, 5), dpi = 80)\n"
            "ax = fig.add_subplot(111)\n"
            "ax.plot(%s,%s,marker='.')\n"
            "ax.set_title(r'%s')\n"
            "plt.xlabel(r'%s')\n"
            "plt.ylabel(r'%s')\n"
            "ax.grid(True)\n"
            "fig.savefig('%s.pdf')\n"
            "fig.savefig('%s.png')\n";

        for (size_t i = 0; i < lst_html_graphs.size(); i++)
        {
            const TGraphData& gd = lst_html_graphs[i];

            const string sOutPy =
                openbeam::format("plot%03u.py", (unsigned int)i);
            const string sOutFile = openbeam::format(
                "%splot%03u", gd.filePrefix.c_str(), (unsigned int)i);
            FILE* f = fopen(sOutPy.c_str(), "wt");
            if (!f)
                throw std::runtime_error(openbeam::format(
                    "can't save output file: `%s`.", sOutPy.c_str()));

            string       sX = "[", sY = "[";
            const size_t N = gd.x.size();
            for (size_t k = 0; k < N; k++)
            {
                sX += openbeam::format("%.3e", gd.x[k]);
                sY += openbeam::format("%.3e", gd.y[k]);
                if (k == (N - 1))
                {
                    sX += "]";
                    sY += "]";
                }
                else
                {
                    sX += ",";
                    sY += ",";
                }
            }

            fprintf(
                f, sPythonTemplate,
                sX.c_str(),  // x data
                sY.c_str(),  // y data
                gd.title.c_str(),  // title
                gd.x_label.c_str(),  // x lb
                gd.y_label.c_str(),  // y lb
                sOutFile.c_str(), sOutFile.c_str());

            fclose(f);

            // Invoke:
            const string sCmd = openbeam::format("python %s", sOutPy.c_str());
            int          ret  = ::system(sCmd.c_str());
            if (ret != 0)
                throw std::runtime_error(openbeam::format(
                    "Error executing cmd: `%s`. Missing Phyton cli?",
                    sCmd.c_str()));
        }
    }

    if (out_html && !out_html_no_head)
    {
        cout << scripts_before_body_end << endl;

        // Footer:
        const string html_tail = openbeam::format(
            "\n<br><br><hr><small>Page generated by OpenBeam - Jose Luis "
            "Blanco Claraco (C) 2016</small><br>"
            "<div align=\"right\">"
            " <a href=\"http://validator.w3.org/check?uri=referer\">"
            " <img border=\"0\" "
            "src=\"http://www.w3.org/Icons/valid-html401-blue\" alt=\"Valid "
            "HTML 4.01 Transitional\" height=\"31\" width=\"88\"></a>\n"
            "</div>\n"
            "</body></html>");

        // HTML end:
        cout << html_tail;
    }

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        return main_code(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
}

template <class MATRIX, class ARRAY>
void print_html_matrix(
    const MATRIX& K, const vector<string>& row_titles,
    const vector<string>& col_titles, const ARRAY* rows_to_use,
    const ARRAY* cols_to_use)
{
    const size_t nCols = K.cols();
    const size_t nRows = K.rows();

    // Determine common factor to make notation clearer:
    int                common_factor_pow = 0;
    std::map<int, int> pow10hits;
    for (size_t i = 0; i < nRows; i++)
    {
        if (rows_to_use && !(*rows_to_use)[i]) continue;

        for (size_t j = 0; j < nCols; j++)
        {
            if (cols_to_use && !(*cols_to_use)[j]) continue;

            const num_t Kijabs = std::abs(K(i, j));

            if (Kijabs > 0)
            {
                const int mag =
                    static_cast<int>(log10(Kijabs));  // Floor to "int"
                pow10hits[mag]++;
            }
            // else: NO, zeroes do not count
        }
    }
    if (!pow10hits.empty())
    {
        map<int, int>::const_iterator it_max = pow10hits.begin();
        for (map<int, int>::const_iterator it = pow10hits.begin();
             it != pow10hits.end(); ++it)
        {
            if (it->second > it_max->second) it_max = it;
        }
        common_factor_pow = it_max->first;
    }

    const num_t common_factor =
        pow(static_cast<num_t>(10), static_cast<num_t>(common_factor_pow));
    const num_t common_factor_inv = 1. / common_factor;

    // Title:
    cout << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";

    // 1st row:
    cout << "<tr>";
    cout << "<td bgcolor=\"#E0E0E0\"> &nbsp; </td>";
    for (size_t i = 0; i < nCols; i++)
    {
        if (cols_to_use && !(*cols_to_use)[i]) continue;

        cout << col_titles[i];
    }
    cout << "</tr>\n";

    // The rest of rows:
    for (size_t i = 0; i < nRows; i++)
    {
        if (rows_to_use && !(*rows_to_use)[i]) continue;

        cout << "<tr>";
        cout << row_titles[i];

        for (size_t j = 0; j < nCols; j++)
        {
            if (cols_to_use && !(*cols_to_use)[j]) continue;

            const num_t Kij = K(i, j);
            if (!Kij)
                cout << format("<td align=\"right\"> 0 </td>");
            else
                cout << format(
                    "<td align=\"right\"> %.3f </td>", Kij * common_factor_inv);
        }

        cout << "</tr>\n";
    }
    cout << "</table>\n";
    cout << format("( &times; 10<sup>%i</sup> )<br>\n", common_factor_pow);
}

template <class MATRIX, class ARRAY>
void print_csv_matrix(
    const std::string& out_csv_file, const MATRIX& K,
    const vector<string>& row_titles, const vector<string>& col_titles,
    const ARRAY* rows_to_use, const ARRAY* cols_to_use)
{
    const size_t nCols = K.cols();
    const size_t nRows = K.rows();

    std::ofstream f(out_csv_file.c_str());
    if (!f.is_open())
        throw std::runtime_error(
            "print_csv_matrix: Error opening output file.");

    // 1st row:
    f << " , ";  // 1st column is a header, skip it.
    for (size_t i = 0; i < nCols; i++)
    {
        if (cols_to_use && !(*cols_to_use)[i]) continue;
        f << col_titles[i];
        if (i < (nCols - 1)) f << " , ";
    }
    f << "\n";

    // The rest of rows:
    for (size_t i = 0; i < nRows; i++)
    {
        if (rows_to_use && !(*rows_to_use)[i]) continue;

        f << row_titles[i] << " , ";

        for (size_t j = 0; j < nCols; j++)
        {
            if (cols_to_use && !(*cols_to_use)[j]) continue;

            f << K(i, j);
            if (j < (nCols - 1)) f << " , ";
        }

        f << "\n";
    }
}

void usedDOFs2titles(
    size_t node_id, vector<string>& titles, bool html,
    const TUsedDoFs* edge_dofs)
{
    for (int k = 0; k < 6; k++)
    {
        if (edge_dofs && !(*edge_dofs)[k]) continue;
        string s;
        if (html) s += "<td>";
        switch (k)
        {
            case 0:
                s += "X";
                break;
            case 1:
                s += "Y";
                break;
            case 2:
                s += "Z";
                break;
            case 3:
                s += "RX";
                break;
            case 4:
                s += "RY";
                break;
            case 5:
                s += "RZ";
                break;
        };
        s += format("<sub>%u</sub>", static_cast<unsigned int>(node_id));
        if (html) s += "</td>";
        titles.push_back(s);
    }
}

void listDOFs2titles(
    const std::vector<TDoF>& dofs, vector<string>& dof_titles, bool html)
{
    const size_t nTot = dofs.size();
    dof_titles.resize(nTot);
    for (size_t i = 0; i < nTot; i++)
    {
        const TDoF& dof = dofs[i];
        string&     s   = dof_titles[i];

        if (html)
            s = format(
                "<td bgcolor=\"#E0E0E0\"> %u (", static_cast<unsigned int>(i));
        else
            s = format("%u (", static_cast<unsigned int>(i));

        switch (dof.dof)
        {
            case 0:
                s += "X";
                break;
            case 1:
                s += "Y";
                break;
            case 2:
                s += "Z";
                break;
            case 3:
                s += "RX";
                break;
            case 4:
                s += "RY";
                break;
            case 5:
                s += "RZ";
                break;
        };

        if (html)
            s += format(
                "<sub>%u</sub>)</td> ", static_cast<unsigned int>(dof.node_id));
        else
            s += format("_%u)", static_cast<unsigned int>(dof.node_id));
    }
}
