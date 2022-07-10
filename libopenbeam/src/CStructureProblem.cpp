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

#include <openbeam/CStructureProblem.h>

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;

CStructureProblem::CStructureProblem() {}

CStructureProblem::~CStructureProblem() { clear(); }

void CStructureProblem::clear()
{
    CFiniteElementProblem::clear();  // Call my base class clear()
    m_loads_on_beams.clear();
}

// Update all internal lists after changing the structure.
void CStructureProblem::updateAll()
{
    // Make sure beams are divided into smaller elements:
    //...

    CFiniteElementProblem::updateAll();  // Call my base class updateAll():
}

void CStructureProblem::addLoadAtBeam(
    const size_t element_index, CLoadOnBeam::Ptr load)
{
    m_loads_on_beams.insert({element_index, load});
}

/** Process loads on elements and populate the \a m_loads_at_each_dof_equivs and
 * \a m_extra_stress_for_each_element
 */
void CStructureProblem::internalComputeStressAndEquivalentLoads()
{
    m_loads_at_each_dof_equivs.clear();  // Clear previous results
    m_extra_stress_for_each_element.clear();

    for (const auto& elIdxLoadPair : m_loads_on_beams)
    {
        auto             el = this->getElement(elIdxLoadPair.first);
        CLoadOnBeam::Ptr l  = elIdxLoadPair.second;

        ElementStress       stress;
        std::vector<array6> loads;

        l->computeStressAndEquivalentLoads(el.get(), stress, loads);

        // Remember: std::map<size_t,num_t> m_loads_at_each_dof_equivs
        //              map: DoF index in the problem -> load value
        ASSERT_(el->conected_nodes_ids.size() == loads.size());

        // For each element edge (two in a typical beam):
        for (size_t p = 0; p < loads.size(); p++)
        {
            const size_t idx_node = el->conected_nodes_ids[p];
            for (int d = 0; d < 6; d++)
            {
                const int dof_idx =
                    m_problem_DoFs_inverse_list[idx_node].dof_index[d];
                if (dof_idx >= 0)
                { m_loads_at_each_dof_equivs[dof_idx] += loads[p][d]; }
                else
                {
                    // Just in case: a load should NOT exists in a DOF which
                    // hasn't been included in the problem:
                    ASSERTDEB_(std::abs(loads[p][d]) < 1e-9)
                }
            }
        }

        // Remember: std::map<size_t, std::vector<array6 > >
        // m_extra_stress_for_each_element;
        //              map: element index -> vector for each "port" -> vector
        //              of 6 stresses values.
        ElementStress& v = m_extra_stress_for_each_element[elIdxLoadPair.first];
        if (v.size() != stress.size())
        {
            // First time, just copy:
            v = stress;
        }
        else
        {
            // Accumulate effects: sum up.
            for (size_t p = 0; p < v.size(); p++) v[p] += stress[p];
        }
    }
}
