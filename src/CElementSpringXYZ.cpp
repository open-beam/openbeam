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

#include <openbeam/CElementSpringXYZ.h>
#include <openbeam/CStructureProblem.h>

using namespace std;
using namespace openbeam;

CElementSpringXYZ::CElementSpringXYZ() : CElement(2), Kx(0), Ky(0), Kz(0) {}

CElementSpringXYZ::CElementSpringXYZ(
    const size_t from_node_id, const size_t to_node_id, const num_t Kx,
    const num_t Ky, const num_t Kz)
    : CElement(2, from_node_id, to_node_id), Kx(Kx), Ky(Ky), Kz(Kz)
{
}

/** Return the stiffness submatrices between each pair of edges in this element,
 * for the current element state.
 */
void CElementSpringXYZ::getLocalStiffnessMatrices(
    openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t& outSubMats)
    const
{
    outSubMats.resize(3);
    TStiffnessSubmatrix& K11 = outSubMats[0];
    TStiffnessSubmatrix& K22 = outSubMats[1];
    TStiffnessSubmatrix& K12 = outSubMats[2];

    // k11 --------------
    K11.edge_in      = 0;
    K11.edge_out     = 0;
    K11.matrix       = TMatrix66::Zero();
    K11.matrix(0, 0) = Kx;
    K11.matrix(1, 1) = Ky;
    K11.matrix(2, 2) = Kz;

    // k22 --------------
    K22.edge_in  = 1;
    K22.edge_out = 1;
    K22.matrix   = K11.matrix;

    // k12 --------------
    K12.edge_in      = 0;
    K12.edge_out     = 1;
    K12.matrix       = TMatrix66::Zero();
    K12.matrix(0, 0) = -Kx;
    K12.matrix(1, 1) = -Ky;
    K12.matrix(2, 2) = -Kz;
}

void CElementSpringXYZ::getLocalDoFs(std::vector<TUsedDoFs>& dofs) const
{
    dofs.resize(getNumberEdges());

    TUsedDoFs sDofs = {Kx != 0, Ky != 0, Kz != 0, false, false, false};
    dofs[0] = dofs[1] = sDofs;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CElementSpringXYZ::loadParamsFromSet(
    const TParamSet& params, const TEvaluationContext& eval)
{
    for (TParamSet::const_iterator it = params.begin(); it != params.end();
         ++it)
    {
        if (strCmpI(it->first, "Kx"))
        { eval.parser_evaluate_expression(it->second, this->Kx); }
        else if (strCmpI(it->first, "Ky"))
        {
            eval.parser_evaluate_expression(it->second, this->Ky);
        }
        else if (strCmpI(it->first, "Kz"))
        {
            eval.parser_evaluate_expression(it->second, this->Kz);
        }
    }
}

#if OPENBEAM_HAS_QT5Svg
void CElementSpringXYZ::drawQtSVG(
    QSvgGenerator& svg, const TDrawStructureOptions& options,
    const TRenderInitData& ri, const TDrawElementExtraParams& draw_el_params,
    const TMeshOutputInfo* meshing_info) const
{
}
#endif

/** Draws the element to a SVG Cairo context (a pointer to a
 * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
 * options */
void CElementSpringXYZ::drawSVG(
    void* _cairo_context, const TDrawStructureOptions& options,
    const TRenderInitData& ri, const TDrawElementExtraParams& draw_el_params,
    const TMeshOutputInfo* meshing_info) const
{
#if OPENBEAM_HAS_CAIRO
    OB_TODO("Implement SVG")
#endif
}

/** Mesh this element into a set of (possibly) smaller ones */
void CElementSpringXYZ::do_mesh(
    const size_t my_idx, CStructureProblem& out_fem, TMeshOutputInfo& out_info,
    const TMeshParams& params)
{
    throw std::runtime_error("TO DO");
}
