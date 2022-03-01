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

#include <openbeam/CElementBeam_2D_AA.h>
#include <openbeam/CElementBeam_2D_AR.h>
#include <openbeam/CElementBeam_2D_RA.h>
#include <openbeam/CElementBeam_2D_RR.h>
#include <openbeam/CStructureProblem.h>

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;

TMeshParams::TMeshParams() : max_element_length(50e-3) {}

void CStructureProblem::mesh(
    CStructureProblem& out_fem, TMeshOutputInfo& out_info,
    const TMeshParams& params)
{
    out_fem.clear();
    out_info = TMeshOutputInfo();

    // ------------------------------
    // (1) NODES: Original ones
    // ------------------------------
    const size_t nOrigNodes     = m_node_poses.size();
    out_info.num_original_nodes = nOrigNodes;
    out_fem.m_node_poses        = this->m_node_poses;
    out_fem.m_node_labels       = this->m_node_labels;

    // ------------------------------
    // (2) NODES: intermediary ones
    // ------------------------------
    const size_t nOrigElements = this->m_elements.size();
    for (size_t i = 0; i < nOrigElements; i++)
        this->m_elements[i]->do_mesh(i, out_fem, out_info, params);

    // ------------------------------
    // (3) Rebuild DOFs lists:
    // ------------------------------
    out_fem.updateAll();

    // ------------------------------
    // (4) Constraints:
    // Copy from the original fem, since they can only appear in the original
    // nodes:
    // ------------------------------
    // Map key are indices in \a m_problem_DoFs.
    //	  *  Map values are displacement in that DoF wrt the current node pose.
    //Units are SI (meters or radians). 	TListOfConstraints  m_DoF_constraints;
    this->updateListDoFs();
    for (TListOfConstraints::const_iterator itC =
             this->m_DoF_constraints.begin();
         itC != this->m_DoF_constraints.end(); ++itC)
    {
        // Get the existing constraint:
        const size_t dof_idx          = itC->first;
        const num_t  constraint_value = itC->second;

        OBASSERT_DEBUG(dof_idx < m_problem_DoFs.size())

        const TDoF&  dof              = m_problem_DoFs[dof_idx];
        const size_t original_node_id = dof.node_id;

        // Insert into the new (meshed) FEM problem:
        const size_t globalIdxDOF = out_fem.getDOFIndex(
            original_node_id, TDoFIndex(dof.dof) /* 0:dx,1:dy,... */);
        OBASSERT(globalIdxDOF != string::npos)

        out_fem.insertConstraint(globalIdxDOF, constraint_value);
    }

    // ------------------------------
    // (3) LOADS: at nodes
    // ------------------------------
    // Since the original nodes are initially copied in the new FEM problem,
    // we can just copy the forces with the same node IDs:
    // out_fem.m_loads_at_each_dof = this->m_loads_at_each_dof;
    for (TListOfLoads::const_iterator it = m_loads_at_each_dof.begin();
         it != m_loads_at_each_dof.end(); ++it)
    {
        // Get the existing load:
        const size_t dof_idx    = it->first;
        const num_t  load_value = it->second;

        OBASSERT_DEBUG(dof_idx < m_problem_DoFs.size())

        const TDoF&  dof              = m_problem_DoFs[dof_idx];
        const size_t original_node_id = dof.node_id;

        // Insert into the new (meshed) FEM problem:
        const size_t globalIdxDOF = out_fem.getDOFIndex(
            original_node_id, TDoFIndex(dof.dof) /* 0:dx,1:dy,... */);
        OBASSERT(globalIdxDOF != string::npos)

        out_fem.setLoadAtDOF(globalIdxDOF, load_value);
    }

    // ------------------------------
    // (4) LOADS: at elements
    // ------------------------------
    for (std::multimap<size_t, CLoadOnBeam*>::const_iterator itLoad =
             m_loads_on_beams.begin();
         itLoad != m_loads_on_beams.end(); ++itLoad)
    {
        const size_t       org_bar_idx = itLoad->first;
        const CLoadOnBeam* load        = itLoad->second;

        load->meshLoad(
            out_fem, out_info.element2elements[org_bar_idx], org_bar_idx,
            *this);
    }
}
