/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2021  Jose Luis Blanco Claraco                       |
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

#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CText.h>
#include <openbeam/CBaseElementBeam.h>
#include <openbeam/CElementBeam_2D_AA.h>
#include <openbeam/CElementBeam_2D_AR.h>
#include <openbeam/CElementBeam_2D_RA.h>
#include <openbeam/CElementBeam_2D_RD.h>
#include <openbeam/CElementBeam_2D_RR.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>
#include <openbeam/DrawStructureOptions.h>

#if OPENBEAM_HAS_CAIRO
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include <cairommconfig.h>
#endif

using namespace std;
using namespace openbeam;

CBaseElementBeam::CBaseElementBeam(
    const bool pinned_end0, const bool pinned_end1)
    : CElement(2), m_pinned_end0(pinned_end0), m_pinned_end1(pinned_end1)
{
}

CBaseElementBeam::CBaseElementBeam(
    const size_t from_node_id, const size_t to_node_id, const bool pinned_end0,
    const bool pinned_end1)
    : CElement(2, from_node_id, to_node_id),
      m_pinned_end0(pinned_end0),
      m_pinned_end1(pinned_end1)
{
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CBaseElementBeam::loadParamsFromSet(
    const mrpt::containers::yaml& p, const EvaluationContext& ctx)
{
    if (p.has("E")) this->E = ctx.evaluate(p["E"]);
    if (p.has("A")) this->A = ctx.evaluate(p["A"]);
    if (p.has("Iz")) this->Iz = ctx.evaluate(p["Iz"]);
    if (p.has("G")) this->G = ctx.evaluate(p["G"]);
    if (p.has("J")) this->J = ctx.evaluate(p["J"]);
}

std::string CBaseElementBeam::asString() const
{
    using namespace std::string_literals;

    std::string ret;
    if (E != UNINITIALIZED_VALUE) ret += "E="s + mrpt::format("%g ", E);
    if (A != UNINITIALIZED_VALUE) ret += "A="s + mrpt::format("%g ", A);
    if (Iz != UNINITIALIZED_VALUE) ret += "Iz="s + mrpt::format("%g ", Iz);
    if (G != UNINITIALIZED_VALUE) ret += "G="s + mrpt::format("%g ", G);
    if (J != UNINITIALIZED_VALUE) ret += "J="s + mrpt::format("%g ", J);

    return ret;
}

mrpt::opengl::CSetOfObjects::Ptr CBaseElementBeam::getVisualization(
    const DrawStructureOptions& o, const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo* meshing_info) const
{
    auto gl = mrpt::opengl::CSetOfObjects::Create();

    const size_t MAX_NODE_ID_TO_DRAW = meshing_info
                                           ? meshing_info->num_original_nodes
                                           : static_cast<size_t>(-1);

    // Draw beam between: node_ids  0 ==> 1
    // Use m_pinned_endX.
    ASSERT_(conected_nodes_ids.size() == 2);

    // Get original or deformed positions:
    // don't draw if it's an "intermediary" node
    // created after meshing
    const bool node0_is_to_draw = conected_nodes_ids[0] < MAX_NODE_ID_TO_DRAW;
    const bool node1_is_to_draw = conected_nodes_ids[1] < MAX_NODE_ID_TO_DRAW;

    TRotationTrans3D p0 = m_parent->getNodePose(conected_nodes_ids[0]);
    TRotationTrans3D p1 = m_parent->getNodePose(conected_nodes_ids[1]);

    if (!draw_el_params.draw_original_position)
    {  // Deformed position:
        Vector3 pt0_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[0], pt0_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        Vector3 pt1_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[1], pt1_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        for (int l = 0; l < 3; l++)
        {
            p0.t[l] = pt0_deformed[l];
            p1.t[l] = pt1_deformed[l];
        }
    }

    // Unit director vector:
    const TPoint3D dir = (p1.t - p0.t).unitarize();

    // Draw the beam itself:
    TPoint3D pt0_end = p0.t;
    TPoint3D pt1_end = p1.t;

    TPoint3D pt0 = pt0_end;
    TPoint3D pt1 = pt1_end;

    const num_t shorten0 =
        node0_is_to_draw
            ? o.NODE_RADIUS + (m_pinned_end0 ? 2 * o.BEAM_PINNED_RADIUS : 0)
            : 0;
    const num_t shorten1 =
        node1_is_to_draw
            ? o.NODE_RADIUS + (m_pinned_end1 ? 2 * o.BEAM_PINNED_RADIUS : 0)
            : 0;

    pt0 += shorten0 * dir;
    pt0_end += o.NODE_RADIUS * dir;

    pt1 -= shorten1 * dir;
    pt1_end -= o.NODE_RADIUS * dir;

    // beam cylinder:
    auto glBody = mrpt::opengl::CArrow::Create();
    glBody->setColor(0.4f, 0.4f, 0.4f, draw_el_params.color_alpha);
    glBody->setArrowEnds(pt0, pt1);
    glBody->setHeadRatio(0);
    glBody->setSmallRadius(o.EDGE_WIDTH);
    gl->insert(glBody);

    // And the "pinned ends circles":
    if (m_pinned_end0 && node0_is_to_draw)
    {
        auto glPin = mrpt::opengl::CSphere::Create();
        glPin->setRadius(o.BEAM_PINNED_RADIUS);
        glPin->setColor_u8(0xf0, 0xf0, 0xf0, 0xff);
        glPin->setNumberDivsLatitude(o.PIN_SPHERE_DIVS);
        glPin->setNumberDivsLongitude(o.PIN_SPHERE_DIVS);
        glPin->setLocation(pt0_end + dir * o.BEAM_PINNED_RADIUS);
        gl->insert(glPin);
    }
    if (m_pinned_end1 && node1_is_to_draw)
    {
        auto glPin = mrpt::opengl::CSphere::Create();
        glPin->setRadius(o.BEAM_PINNED_RADIUS);
        glPin->setColor_u8(0xf0, 0xf0, 0xf0, 0xff);
        glPin->setNumberDivsLatitude(o.PIN_SPHERE_DIVS);
        glPin->setNumberDivsLongitude(o.PIN_SPHERE_DIVS);
        glPin->setLocation(pt1_end - dir * o.BEAM_PINNED_RADIUS);
        gl->insert(glPin);
    }

    if (o.show_element_labels && node0_is_to_draw && node1_is_to_draw)
    {
        const auto p =
            ((p0.t + p1.t) * 0.5) + 0.5 * o.NODE_RADIUS * TPoint3D(1, 1, 1);

        auto glLb = mrpt::opengl::CText::Create();
        glLb->setColor_u8(0x00, 0x00, 0xd0);
        glLb->setLocation(p);
        glLb->setString(mrpt::format(
            "E%u", static_cast<unsigned int>(draw_el_params.element_index)));
        gl->insert(glLb);
    }

    return gl;
}

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CBaseElementBeam::drawSVG(
    void* _cairo_context, const DrawStructureOptions& options,
    const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    Cairo::RefPtr<Cairo::Context>& cr =
        *reinterpret_cast<Cairo::RefPtr<Cairo::Context>*>(_cairo_context);

    const double EDGE_WIDTH          = 5 / ri.scaleFactor;  // 2e-2;
    const double BEAM_PINNED_RADIUS  = 4 / ri.scaleFactor;  // 3e-2;
    const double PINNED_PEN_WIDTH    = 0.05 * BEAM_PINNED_RADIUS;
    const size_t MAX_NODE_ID_TO_DRAW = meshing_info
                                           ? meshing_info->num_original_nodes
                                           : static_cast<size_t>(-1);

    // Draw beam between: node_ids  0 ==> 1
    // Use m_pinned_endX.
    ASSERT_(conected_nodes_ids.size() == 2);

    // Get original or deformed positions:
    const bool node0_is_to_draw =
        conected_nodes_ids[0] <
        MAX_NODE_ID_TO_DRAW;  // don't draw if it's an "intermediary" node
                              // created after meshing
    const bool node1_is_to_draw = conected_nodes_ids[1] < MAX_NODE_ID_TO_DRAW;

    TRotationTrans3D p0 = m_parent->getNodePose(conected_nodes_ids[0]);
    TRotationTrans3D p1 = m_parent->getNodePose(conected_nodes_ids[1]);
    if (!draw_el_params.draw_original_position)
    {  // Deformed position:
        Vector3 pt0_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[0], pt0_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        Vector3 pt1_deformed;
        m_parent->getNodeDeformedPosition(
            conected_nodes_ids[1], pt1_deformed, *draw_el_params.solver_info,
            draw_el_params.deformed_scale_factor);

        for (int l = 0; l < 3; l++)
        {
            p0.t[l] = pt0_deformed[l];
            p1.t[l] = pt1_deformed[l];
        }
    }

    // Unit director vector:
    const TPoint3D dir = (p1.t - p0.t).unitarize();

    // Draw the beam itself:
    TPoint3D pt0_end = TPoint3D(p0.t.x, p0.t.y, p0.t.z);
    TPoint3D pt1_end = TPoint3D(p1.t.x, p1.t.y, p1.t.z);

    TPoint3D pt0 = pt0_end;
    TPoint3D pt1 = pt1_end;

    const num_t shorten0 =
        node0_is_to_draw
            ? options.NODE_RADIUS + (m_pinned_end0 ? 2 * BEAM_PINNED_RADIUS : 0)
            : 0;
    const num_t shorten1 =
        node1_is_to_draw
            ? options.NODE_RADIUS + (m_pinned_end1 ? 2 * BEAM_PINNED_RADIUS : 0)
            : 0;

    pt0 += shorten0 * dir;
    pt0_end += options.NODE_RADIUS * dir;

    pt1 -= shorten1 * dir;
    pt1_end -= options.NODE_RADIUS * dir;

    cr->set_source_rgba(0.4, 0.4, 0.4, draw_el_params.color_alpha);
    cr->set_line_width(EDGE_WIDTH);
    cr->move_to(pt0.x, pt0.y);
    cr->line_to(pt1.x, pt1.y);
    cr->stroke();

    // And the "pinned ends circles":
    if (m_pinned_end0 && node0_is_to_draw)
    {
        cr->set_line_width(PINNED_PEN_WIDTH);
        cr->arc(
            pt0_end.x + dir.x * BEAM_PINNED_RADIUS,
            pt0_end.y + dir.y * BEAM_PINNED_RADIUS, BEAM_PINNED_RADIUS, 0,
            2 * M_PI);
        cr->stroke();
    }
    if (m_pinned_end1 && node1_is_to_draw)
    {
        cr->set_line_width(PINNED_PEN_WIDTH);
        cr->arc(
            pt1_end.x - dir.x * BEAM_PINNED_RADIUS,
            pt1_end.y - dir.y * BEAM_PINNED_RADIUS, BEAM_PINNED_RADIUS, 0,
            2 * M_PI);
        cr->stroke();
    }

    if (options.show_element_labels && node0_is_to_draw && node1_is_to_draw)
    {
        cr->set_source_rgb(0, 0, 0.9);

        const double x0 = 0.5 * (p0.t.x + p1.t.x) + 0.5 * options.NODE_RADIUS;
        const double y0 = 0.5 * (p0.t.y + p1.t.y) + 0.5 * options.NODE_RADIUS;
        cr->move_to(x0, y0);
        cr->show_text(openbeam::format(
            "E%u", static_cast<unsigned int>(draw_el_params.element_index)));
    }

#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CBaseElementBeam::do_mesh(
    const size_t my_idx, CStructureProblem& out_fem, MeshOutputInfo& out_info,
    const MeshParams& params)
{
    // Compute my current length by getting the poses of the two nodes at each
    // end of the beam:
    ASSERTDEB_(conected_nodes_ids.size() == 2)
    ASSERTDEB_(m_parent != nullptr)

    const TRotationTrans3D& node0 =
        m_parent->getNodePose(conected_nodes_ids[0]);
    const TRotationTrans3D& node1 =
        m_parent->getNodePose(conected_nodes_ids[1]);

    // vector: 0->1
    const TPoint3D v01(
        node1.t.x - node0.t.x, node1.t.y - node0.t.y, node1.t.z - node0.t.z);
    const num_t L = v01.norm();

    ASSERT_(L > 0);
    ASSERT_(params.max_element_length > 0);

    // Compute # of elements:
    const size_t nElements =
        static_cast<size_t>(ceil(L / params.max_element_length));

    // 1/2: Create new intermediary nodes:
    // -----------------------------------------------
    if (out_info.element2nodes.size() <= my_idx)
        out_info.element2nodes.resize(my_idx + 1);

    std::vector<size_t>& my_nodes = out_info.element2nodes[my_idx];
    ASSERTDEB_(my_nodes.empty())

    my_nodes.reserve(nElements + 1);
    my_nodes.push_back(conected_nodes_ids[0]);
    for (size_t node_idx = 1; node_idx < nElements;
         node_idx++)  //  [0,1,..,N-1,N]:  N elements -> N+1 nodes:
    {
        TRotationTrans3D p;
        p.r            = node0.r;
        const double f = node_idx / double(nElements);
        for (int i = 0; i < 3; i++) p.t[i] = node0.t[i] + v01[i] * f;

        const size_t new_node_id = out_fem.insertNode(p);
        my_nodes.push_back(new_node_id);
    }
    my_nodes.push_back(conected_nodes_ids[1]);

    // 2/2: Create new elements:
    // -----------------------------------------------
    if (out_info.element2elements.size() <= my_idx)
        out_info.element2elements.resize(my_idx + 1);

    std::vector<size_t>& my_elements = out_info.element2elements[my_idx];
    ASSERTDEB_(my_elements.empty())
    my_elements.reserve(nElements);

    for (size_t idx_el = 0; idx_el < nElements; idx_el++)
    {
        CBaseElementBeam::Ptr new_el;
        const size_t          ni = my_nodes[idx_el];
        const size_t          nj = my_nodes[idx_el + 1];
        if (idx_el == 0)
        {
            OB_TODO(
                "Make virtual method to allow each element to tell its special "
                "first and ending element.");
            // First element:
            if (dynamic_cast<CElementBeam_2D_AR*>(this) != nullptr ||
                dynamic_cast<CElementBeam_2D_AA*>(this) != nullptr)
                new_el = std::make_shared<CElementBeam_2D_AR>(ni, nj);
            //			else if
            //(dynamic_cast<CElementBeam_2D_DR*>(this)!=nullptr)
            // new_el = new CElementBeam_2D_DR(ni,nj);
            else
            {
                new_el = std::make_shared<CElementBeam_2D_RR>(ni, nj);
            }
        }
        else if (idx_el == nElements - 1)
        {
            // Last element:
            if (dynamic_cast<CElementBeam_2D_RA*>(this) != nullptr ||
                dynamic_cast<CElementBeam_2D_AA*>(this) != nullptr)
                new_el = std::make_shared<CElementBeam_2D_RA>(ni, nj);
            else if (dynamic_cast<CElementBeam_2D_RD*>(this) != nullptr)
                new_el = std::make_shared<CElementBeam_2D_RD>(ni, nj);
            else
            {
                new_el = std::make_shared<CElementBeam_2D_RR>(ni, nj);
            }
        }
        else
        {
            // The rest of (intermediary) elements:
            new_el = std::make_shared<CElementBeam_2D_RR>(ni, nj);
        }
        // Copy params:
        new_el->copyCommonBeamParamsFrom(*this);

        // Insert:
        size_t new_el_idx = out_fem.insertElement(new_el);
        my_elements.push_back(new_el_idx);
    }
}
