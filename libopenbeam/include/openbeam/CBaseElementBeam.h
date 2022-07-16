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

#pragma once

#include "CElement.h"
#include "types.h"

namespace openbeam
{
/** A base class for Beam elements; not usable as an standalone element. */
class CBaseElementBeam : public CElement
{
   public:
    using Ptr      = std::shared_ptr<CBaseElementBeam>;
    using ConstPtr = std::shared_ptr<const CBaseElementBeam>;

    CBaseElementBeam(const bool pinned_end0, const bool pinned_end1);
    CBaseElementBeam(
        const size_t from_node_id, const size_t to_node_id,
        const bool pinned_end0, const bool pinned_end1);

    num_t E  = UNINITIALIZED_VALUE;  //!< Young modulus (N/m^2)
    num_t A  = UNINITIALIZED_VALUE;  //!< Section (m^2)
    num_t Iz = UNINITIALIZED_VALUE;  //!< Section inertia moment (m^4)
    num_t G  = 0;  //!< Shear modulus of elasticity (N/m^2)
    num_t J  = 0;  //!< Polar moment of inertia (m^4)

    /** Sets the basic beam parameters from another existing element. */
    void copyCommonBeamParamsFrom(const CBaseElementBeam& o)
    {
        E  = o.E;
        A  = o.A;
        Iz = o.Iz;
        G  = o.G;
        J  = o.J;
    }

    std::string asString() const override;

    /** Draws the element to a SVG Cairo context (a pointer to a
     * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
     * options */
    void drawSVG(
        void* _cairo_context, const DrawStructureOptions& options,
        const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo* meshing_info) const override;
#if OPENBEAM_HAS_QT5Svg
    void drawQtSVG(
        QSvgGenerator& svg, const DrawStructureOptions& options,
        const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo* meshing_info) const override;
#endif

    /** Mesh this element into a set of (possibly) smaller ones */
    void do_mesh(
        const size_t my_idx, CStructureProblem& out_fem,
        MeshOutputInfo& out_info, const MeshParams& params) override;

    /** Parse a set of parameters by (casi insensitive) name and set the element
     * values from them.
     *  Each element must document the supported parameters and their meaning.
     */
    void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx) override;

   private:
    bool m_pinned_end0,
        m_pinned_end1;  //!< Whether each end of the beam is "pinned" (i.e.
                        //!< allows free rotation). This is only used for
                        //!< drawing purposes
};
}  // namespace openbeam
