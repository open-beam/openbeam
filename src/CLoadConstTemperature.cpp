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

#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>
#include <openbeam/elements.h>
#include <openbeam/loads.h>

using namespace std;
using namespace openbeam;

void CLoadConstTemperature::computeStressAndEquivalentLoads(
    const CElement* el, TElementStress& stress, std::vector<array6>& loads)
{
    // Stress -------------------------------------------
    //  Axial only: N = E A alpha AT
    num_t E, A, alpha;

    if (dynamic_cast<const CBaseElementBeam*>(el) != NULL)
    {
        const CBaseElementBeam* e = dynamic_cast<const CBaseElementBeam*>(el);
        E                         = e->E;
        A                         = e->A;
        alpha                     = 12e-6;  // e->params.alpha;
    }
    else
        throw std::runtime_error(
            "Unsupported element type for load 'CLoadConstTemperature'");

    //  Axial:
    const num_t N = E * A * alpha * this->incr_temp;
    stress.resize(2);

    // S1 = [-N, 0, 0 , 0, 0, 0]
    stress[0].N = -N;

    // S2 = [-N, 0, 0 , 0, 0, 0]
    stress[1].N = -N;

    // Equivalent load -------------------------------------------
    loads.resize(2);
    for (int i = 0; i < 6; i++)
    {
        loads[0][i] = 0;
        loads[1][i] = 0;
    }

    const TRotationTrans3D& node0 =
        el->getParent()->getNodePose(el->conected_nodes_ids[0]);
    const TRotationTrans3D& node1 =
        el->getParent()->getNodePose(el->conected_nodes_ids[1]);

    num_t Ax = node1.t.coords[0] - node0.t.coords[0];
    num_t Ay = node1.t.coords[1] - node0.t.coords[1];
    num_t Az = node1.t.coords[2] - node0.t.coords[2];

    const num_t L2 = square(Ax) + square(Ay) + square(Az);
    const num_t L  = std::sqrt(L2);
    OBASSERT(L > 0)
    const num_t _1_L = 1 / L;

    // Normalize direction vector:
    Ax *= _1_L;
    Ay *= _1_L;
    Az *= _1_L;

    loads[0][0] = -N * Ax;
    loads[0][1] = -N * Ay;
    loads[0][2] = -N * Az;

    loads[1][0] = N * Ax;
    loads[1][1] = N * Ay;
    loads[1][2] = N * Az;
}

/** See declaration in base class */
void CLoadConstTemperature::loadParamsFromSet(
    const TParamSet& params, const TEvaluationContext& eval)
{
    for (TParamSet::const_iterator it = params.begin(); it != params.end();
         ++it)
    {
        if (strCmpI(it->first, "deltaT"))
        { eval.parser_evaluate_expression(it->second, this->incr_temp); }
        else
        {
            if (eval.warn_msgs)
                eval.warn_msgs->push_back(format(
                    "*Warning* Ignoring unknown parameter %s",
                    it->first.c_str()));
        }
    }
}

/** Decompose the distributed load as needed into the set of elements in which
 * the original element has been meshed */
void CLoadConstTemperature::meshLoad(
    CStructureProblem&         meshed_fem,
    const std::vector<size_t>& meshed_element_idxs,
    const size_t original_bar_idx, const CStructureProblem& original_fem) const
{
    // Temperature loads are just decomposed into identical loads at each
    // element:
    for (size_t i = 0; i < meshed_element_idxs.size(); i++)
        meshed_fem.addLoadAtBeam(
            meshed_element_idxs[i], new CLoadConstTemperature(this->incr_temp));
}
