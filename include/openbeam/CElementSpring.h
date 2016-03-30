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


#pragma once

#include "types.h"
#include "CElement.h"

namespace openbeam
{
	/** A spring element in the local X direction between two given nodes.
	  */
	class CElementSpring : public CElement
	{
	public:
		CElementSpring();
		CElementSpring(const size_t from_node_id, const size_t to_node_id, const num_t K = 1e7);

		/** Return the stiffness submatrices between each pair of edges in this element, for the current element state.
		  */
		virtual void getLocalStiffnessMatrices( openbeam::aligned_containers<TStiffnessSubmatrix>::vector_t  &outSubMats ) const ;

		virtual void getLocalDoFs(std::vector<TUsedDoFs> &dofs) const;

		num_t    K;   //!< Stiffness constant of the spring (N/m)

		/** Parse a set of parameters by (casi insensitive) name and set the element values from them.
		  *  Each element must document the supported parameters and their meaning.  */
		virtual void loadParamsFromSet( const TParamSet & params, const TEvaluationContext &eval);

		/** Draws the element to a SVG Cairo context (a pointer to a Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed options */
		virtual void drawSVG(void *_cairo_context,const TDrawStructureOptions &options,  const TRenderInitData & ri,const TDrawElementExtraParams &draw_el_params,const TMeshOutputInfo *meshing_info) const;


		/** Mesh this element into a set of (possibly) smaller ones */
		virtual void do_mesh(const size_t my_idx, CStructureProblem &out_fem, TMeshOutputInfo &out_info, const TMeshParams & params);
	};
}
