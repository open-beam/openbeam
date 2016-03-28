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
   +---------------------------------------------------------------------------+ */


#include <openbeam/CFiniteElementProblem.h>

#include <set>

using namespace std;
using namespace openbeam;
using namespace Eigen;


CFiniteElementProblem::CFiniteElementProblem()
{


}

CFiniteElementProblem::~CFiniteElementProblem()
{
	clear();
}


void CFiniteElementProblem::clear()
{
    m_node_poses.clear();
    m_DoF_constraints.clear();
	m_loads_at_each_dof.clear();
	m_loads_at_each_dof_equivs.clear();
	m_extra_stress_for_each_element.clear();
    free_container(m_elements);
}

size_t CFiniteElementProblem::insertElement( CElement* el )
{
	OBASSERT(el!=NULL)
    el->setParent(this);
	const size_t idx = m_elements.size();
    m_elements.push_back(el);
	return idx;
}

/** Return a constant pointer to the i'th element in the structure. */
const CElement* CFiniteElementProblem::getElement(size_t i) const
{
	OBASSERT(i<m_elements.size())
	return  m_elements[i];
}

/** Return a pointer to the i'th element in the structure. */
CElement* CFiniteElementProblem::getElement(size_t i)
{
	OBASSERT(i<m_elements.size())
	return  m_elements[i];
}

/** Insert a new constraint in the structure (this objects owns the memory, will delete the element object when not needed anymore).
* \return The constraint index */
void CFiniteElementProblem::insertConstraint( const size_t dof_index, const num_t value )
{
	if (m_problem_DoFs.empty()) throw std::runtime_error("insertConstraint(): Empty DOFs (Have you called updateAll() first?)");
	OBASSERT(dof_index<m_problem_DoFs.size())
	m_DoF_constraints[dof_index] = value;
}

/** Sets the load (force or moment) applied to a given DOF  \sa addLoadAtDOF */
void CFiniteElementProblem::setLoadAtDOF( const size_t dof_index, const num_t f)
{
	if (m_problem_DoFs.empty()) throw std::runtime_error("setLoadAtDOF(): Empty DOFs (Have you called updateAll() first?)");
	OBASSERT(dof_index<m_problem_DoFs.size())
	m_loads_at_each_dof[dof_index] = f;
}

/** Adds a load (force or moment) to a given DOF, accumulating to previously existing values if any.  \sa setLoadAtDOF */
void CFiniteElementProblem::addLoadAtDOF( const size_t dof_index, const num_t f)
{
	if (m_problem_DoFs.empty()) throw std::runtime_error("addLoadAtDOF(): Empty DOFs (Have you called updateAll() first?)");
	OBASSERT(dof_index<m_problem_DoFs.size())
	m_loads_at_each_dof[dof_index] += f;
}




void CFiniteElementProblem::setNumberOfNodes(size_t N)
{
	m_node_poses.resize(N);
	m_node_labels.resize(N);
}

void CFiniteElementProblem::setNodePose(size_t idx, const TRotationTrans3D &p)
{
	OBASSERT(idx<m_node_poses.size())
    m_node_poses[idx] = p;
}

void CFiniteElementProblem::setNodePose(size_t idx, const num_t x, const num_t y, const num_t z)
{
	OBASSERT(idx<m_node_poses.size())

    TRotationTrans3D &P = m_node_poses[idx];
    P.t.coords[0] = x;
    P.t.coords[1] = y;
    P.t.coords[2] = z;
}

size_t CFiniteElementProblem::insertNode(const TRotationTrans3D &p)
{
	const size_t idx = m_node_poses.size();
    m_node_poses.push_back(p);
	return idx;
}


// Update all internal lists after changing the structure.
void CFiniteElementProblem::updateAll()
{
	updateElementsOrientation();
    updateListDoFs();
}

/** Call the "updateOrientationFromNodePositions" in each element */
void CFiniteElementProblem::updateElementsOrientation()
{
    const size_t nElements = m_elements.size();
	for (size_t i=0;i<nElements;i++)
		m_elements[i]->updateOrientationFromNodePositions();

}


/** From the list of elements and their properties and connections, build the list of DoFs relevant to the problem.
*/
void CFiniteElementProblem::updateListDoFs()
{
    updateNodeConnections();  // We need these data OK. (m_node_connections)

    { // Real clear of "m_problem_DoFs"
        std::vector<TDoF> dum;
        m_problem_DoFs.swap(dum);
    }

	// Go thru nodes, and add those DoF that appear at least once:
    const size_t nNodes    = m_node_poses.size();
	OBASSERT(nNodes==m_node_connections.size())

	// Start with a list of all DOFs marked as unused:
	m_problem_DoFs_inverse_list.assign(nNodes, TProblemDOFIndicesForNode() );

    for (size_t i=0;i<nNodes;i++)
	{
		TUsedDoFs dofs;
		for (unsigned char d=0;d<6;d++) dofs[d]=false;

		for (TNodeConnections::const_iterator it=m_node_connections[i].begin();it!=m_node_connections[i].end();++it)
			for (int d=0;d<6;d++)
				if (it->second.dofs[d])
					dofs[d]=true;

		for (unsigned char d=0;d<6;d++)
			if (dofs[d])
			{
				const int new_dof_index = static_cast<int>(m_problem_DoFs.size());

				m_problem_DoFs.push_back( TDoF(i,d) );
				m_problem_DoFs_inverse_list[i].dof_index[d] = new_dof_index;
			}
	}

}

/** From the list of elements, update m_node_connections */
void CFiniteElementProblem::updateNodeConnections()
{
    // Reset list of info per node:
    const size_t nElements = m_elements.size();
    const size_t nNodes    = m_node_poses.size();

    m_node_connections.assign(nNodes, TNodeConnections() );

    for (size_t i=0;i<nElements;i++)
    {
        const CElement   *el = m_elements[i];
        const size_t nElementFaces = el->conected_nodes_ids.size();

        vector<TUsedDoFs> el_dofs;
        //el->getLocalDoFs(el_dofs);
		el->getGlobalDoFs(el_dofs);

        for (size_t k=0;k<nElementFaces;k++)
        {
            OBASSERT( el->conected_nodes_ids[k]<nNodes)
            TNodeConnections &nc = m_node_connections[el->conected_nodes_ids[k]];
            nc[i].element_face_id = static_cast<unsigned char>(k);
            nc[i].dofs = el_dofs[k];
        }
    }
}

// Report all the problem DOFs as text:
std::string CFiniteElementProblem::getProblemDoFsDescription()
{
	std::stringstream  s;

	updateListDoFs();
	for (size_t i=0;i<m_problem_DoFs.size();i++)
	{
		s << "DoF #"  << i << ": nodeID= " << m_problem_DoFs[i].node_id  << " DoF=" ;
		switch(m_problem_DoFs[i].dof)
		{
		case 0: s << "x\n"; break;
		case 1: s << "y\n"; break;
		case 2: s << "z\n"; break;
		case 3: s << "thx\n"; break;
		case 4: s << "thy\n"; break;
		case 5: s << "thz\n"; break;
		default: break;
		}
	}

	return s.str();
}

/** Return the complementary list of DOFs, such as "return \CUP ds = 1:N" and "return \CAP ds = \empty".
  */
std::vector<size_t>  CFiniteElementProblem::complementaryDoFs(
	const std::vector<size_t> &ds,
	const size_t nTotalDOFs)
{
	// Convert input list into a set:
	std::set<size_t> sv;
	for (size_t i=0;i<ds.size();i++)
		sv.insert(ds[i]);


	std::vector<size_t> v;
	v.reserve(nTotalDOFs-sv.size());

	for (size_t i=0;i<nTotalDOFs;i++)
		if (sv.find(i)==sv.end())
			v.push_back(i);

	return v;
}

/** Returns the 0-based index of the n'th DOF (0-5) of node "nNode" in the list of DOFs to consider in the problem.
	*  Callable only after \a buildAll().
	*  Returns string::npos if the DOF is NOT considered in the problem.
	*/
size_t CFiniteElementProblem::getDOFIndex(const size_t nNode, const TDoFIndex n) const
{
	int idx = m_problem_DoFs_inverse_list[nNode].dof_index[n];
	if (idx<0) return std::string::npos;
	else return static_cast<size_t>(idx);
}


/** Returns the 3D location of the final deformed state after solving the problem */
void CFiniteElementProblem::getNodeDeformedPosition(
	size_t i,
	TVector3 &out_final_point,
	const TStaticSolveProblemInfo & solver_info,
	const num_t exageration_factor) const
{
	OBASSERT_DEBUG(i<m_node_poses.size())

	TVector3 incr_pt; // Increment in each (x,y,z) coordinate for this node:

	const TProblemDOFIndicesForNode &DOFs_i = this->m_problem_DoFs_inverse_list[i];
	for (int l=0;l<3;l++)  // go thru DOFs: x, y, z
	{
		if (DOFs_i.dof_index[l]<0)
		{
			incr_pt[l] = 0; // DOF not considered in the problem.
		}
		else
		{
			const size_t dof_index = DOFs_i.dof_index[l];
			// Only one of the indices will be valid:
			const size_t b_idx = solver_info.build_info.dof_types[dof_index].bounded_index;
			const size_t f_idx = solver_info.build_info.dof_types[dof_index].free_index;

			if (b_idx==std::string::npos)
					incr_pt[l] = solver_info.U_f[f_idx];
			else	incr_pt[l] = solver_info.build_info.U_b[b_idx];
		}
	}

	// Compose position =============
	// Rotation of increment:
	const TRotationTrans3D & original_pose = m_node_poses[i];
	out_final_point = original_pose.r.getRot() * incr_pt * exageration_factor;

	// Plus original translation:
	for (int l=0;l<3;l++)
		out_final_point[l]+=original_pose.t.coords[l];
}

/** Returns the maximum absolute value translation in any X,Y,Z directions
     for a static problem solution */
num_t CFiniteElementProblem::getMaximumDeformedDisplacement(const TStaticSolveProblemInfo & solver_info) const
{
	num_t cur_max = 0;

	for (size_t i=0;i<m_problem_DoFs_inverse_list.size();i++)
	{
		const TProblemDOFIndicesForNode &DOFs_i = this->m_problem_DoFs_inverse_list[i];
		for (int l=0;l<3;l++)  // go thru DOFs: x, y, z
		{
			if (DOFs_i.dof_index[l]<0) continue;  // DOF not considered in the problem.

			const size_t dof_index = DOFs_i.dof_index[l];
			// Only one of the indices will be valid:
			const size_t b_idx = solver_info.build_info.dof_types[dof_index].bounded_index;
			const size_t f_idx = solver_info.build_info.dof_types[dof_index].free_index;

			num_t displ;
			if (b_idx==std::string::npos)
					displ = solver_info.U_f[f_idx];
			else	displ = solver_info.build_info.U_b[b_idx];

			displ = std::abs(displ);
			if (displ>cur_max)
				cur_max = displ;
		}
	}
	return cur_max;
}

/** Computes the bounding box of all existing nodes */
void CFiniteElementProblem::getBoundingBox(num_t &min_x, num_t &max_x, num_t &min_y ,num_t &max_y) const
{
	min_x = min_y =  std::numeric_limits<num_t>::max();
	max_x = max_y = -std::numeric_limits<num_t>::max();

	const size_t nNodes = getNumberOfNodes();
	for (size_t i=0;i<nNodes;i++)
	{
		const TRotationTrans3D &p = this->getNodePose(i);
		min_x=std::min(min_x, p.t.coords[0]);
		max_x=std::max(max_x, p.t.coords[0]);

		min_y=std::min(min_y, p.t.coords[1]);
		max_y=std::max(max_y, p.t.coords[1]);
	}
}

std::string CFiniteElementProblem::getNodeLabel(const size_t idx) const
{
	if (m_node_labels[idx].empty()) {
		return openbeam::format("N%u",static_cast<unsigned int>(idx));
	}
	else return m_node_labels[idx];
}
