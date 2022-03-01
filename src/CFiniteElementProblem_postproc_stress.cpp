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

#include <openbeam/CFiniteElementProblem.h>

using namespace std;
using namespace openbeam;
using namespace Eigen;

/** After solving the problem with \a solveStatic, you can optionally call this
 * post-processing method to evaluate the stress of all the elements.
 */
void CFiniteElementProblem::postProcCalcStress(
    TStressInfo& out_stress, const TStaticSolveProblemInfo& solver_info)
{
    timelog.enter("postProcCalcStress");

    const size_t nE = m_elements.size();

    // first, make sure we clear the output "element_stress":
    {
        std::vector<TElementStress> dumm;
        out_stress.element_stress.swap(dumm);
    }

    out_stress.element_stress.resize(nE);

    // For convenience, firstly build a simple list with the global displacement
    // of all DOFs,
    //  no matter being free or bounded:
    const size_t                           nNodes = m_node_poses.size();
    aligned_containers<TVector6>::vector_t U(nNodes);
    // First only the zeros (ignored DoFs)
    for (size_t n = 0; n < nNodes; n++)
    {
        const TProblemDOFIndicesForNode& dofs = m_problem_DoFs_inverse_list[n];
        for (size_t k = 0; k < 6; k++)
        {
            const int dof_idx = dofs.dof_index[k];
            if (dof_idx < 0)
            {  // DOF not in the problem:
                U[n][k] = 0;
            }
        }
    }
    const size_t nRestrDOFs = solver_info.build_info.bounded_dof_indices.size();
    for (size_t i = 0; i < nRestrDOFs; i++)
    {
        const size_t dof_idx = solver_info.build_info.bounded_dof_indices[i];
        const TDoF&  D       = m_problem_DoFs[dof_idx];
        U[D.node_id][D.dof]  = solver_info.build_info.U_b[i];
    }
    const size_t nFreeDOFs = solver_info.build_info.free_dof_indices.size();
    for (size_t i = 0; i < nFreeDOFs; i++)
    {
        const size_t dof_idx = solver_info.build_info.free_dof_indices[i];
        const TDoF&  D       = m_problem_DoFs[dof_idx];
        U[D.node_id][D.dof]  = solver_info.U_f[i];
    }

    // For each element, ask for its stiffness sub-matrices, gather the
    //  global displacements of each edge and build a complete
    //  global displacement vector for the element (Ue), then
    //  convert it into local coordinates (Uel), and
    //  left-multiply it appropriately by the stiffness matrices.
    //
    for (size_t i = 0; i < nE; i++)
    {
        CElement*    el     = m_elements[i];
        const size_t nFaces = el->conected_nodes_ids.size();

        // For each face, get the computed displacements of
        //  the corresponding node (Ue)
        //   and convert them to local coords (Uel):
        aligned_containers<TVector6>::vector_t Uel(nFaces);
        for (size_t f = 0; f < nFaces; f++)
        {
            const TVector6& Uf = U[el->conected_nodes_ids[f]];

            // 3 translational elements: (x,y,z):
            Uel[f].block(0, 0, 3, 1).noalias() =
                el->getGlobalOrientation().getRot().transpose() *
                Uf.block(0, 0, 3, 1);

            // 3 rotational elements: (Rx,Ry,Rz):
#if 0
			const TRotation3D incr_rot(Uf[3],Uf[4],Uf[5]);
			const TMatrix33 Rot_local = el->getGlobalOrientation().getRot().transpose() * incr_rot.getRot();
			TRotation3D::matrix2angles(Rot_local,
				Uel[f][3], // Rx : roll
				Uel[f][4], // Ry : roll
				Uel[f][5] // Rz : roll
				);
#else
            OB_TODO(
                "This only works for planar (Z=0) structures!!! Debug formulas "
                "above")
            Uel[f][3] = Uf[3];
            Uel[f][4] = 0;
            Uel[f][5] = Uf[5];
#endif
        }

        // Get local stiffness matrices:
        openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t subMats;
        el->getLocalStiffnessMatrices(subMats);

        /*
         * | f_0 |   | K00  K01 |  | u0l |
         * | f_1 | = | K10  K11 |  | u1l |
         *
         */
        TElementStress& es = out_stress.element_stress[i];
        es.resize(nFaces);

        for (size_t k = 0; k < subMats.size(); k++)
        {
            const TStiffnessSubmatrix& ss = subMats[k];

            if (ss.edge_in == ss.edge_out)
            { es[ss.edge_in] += ss.matrix * Uel[ss.edge_out]; }
            else
            {
                es[ss.edge_in] += ss.matrix * Uel[ss.edge_out];
                es[ss.edge_out] += ss.matrix.transpose() * Uel[ss.edge_in];
            }
        }
    }

    // Change local coordinates to "strength of materials", with our sign
    // conventions:
    //
    for (size_t i = 0; i < nE; i++)
    {
        TElementStress& es = out_stress.element_stress[i];

        if (!es.empty())
        {
            es[0].N  = -es[0].N;
            es[0].Vy = -es[0].Vy;
            es[0].Vz = -es[0].Vz;
            es[0].Mx = -es[0].Mx;
            es[0].My = -es[0].My;
            es[0].Mz = -es[0].Mz;
        }
    }

    // Don't forget to add those stress values in
    // "m_extra_stress_for_each_element;
    for (std::map<size_t, TElementStress>::const_iterator it =
             m_extra_stress_for_each_element.begin();
         it != m_extra_stress_for_each_element.end(); ++it)
    {
        OBASSERT(it->first < nE)
        TElementStress& es = out_stress.element_stress[it->first];

        const TElementStress& es2add = it->second;
        OBASSERT(es.size() == es2add.size())

        for (size_t i = 0; i < es.size(); ++i) es[i] += es2add[i];
    }

    timelog.leave("postProcCalcStress");
}
