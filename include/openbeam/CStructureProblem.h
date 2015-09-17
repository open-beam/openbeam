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

#include "CFiniteElementProblem.h"
#include "loads.h"


namespace openbeam
{

	/** A complete structure problem: it extends a finite-element problem by providing automatic division of large beams into sequence of small elements, etc.
	  *
	  */
	class CStructureProblem : public CFiniteElementProblem
	{
	public:
        CStructureProblem();
        virtual ~CStructureProblem();

        virtual void clear(); //!< Delete all elements, nodes and constraints in this structure, completely emptying it.

		/** Adds a new load to an element. The object must be created with "new", and not deleted by the user (it'll be done automatically by this class).
		  * \sa CFiniteElementProblem, CFiniteElementProblem::addLoadAtDOF
		  */
		void addLoadAtBeam(const size_t element_index, CLoadOnBeam* load);

		// ----------------------------------------------------------------------------
		/** @name Structure solving
		    @{ */

        virtual void updateAll(); //!< Update all internal lists after changing the structure.

		/** Mesh the structure into a set of small elements.
		  */
		void mesh( 
			CStructureProblem &out_fem,
			TMeshOutputInfo &out_info,
			const TMeshParams & params
			); 

		/** @} */
		// ----------------------------------------------------------------------------

	private:
        /** @name Structure data
            @{ */
		std::multimap<size_t,CLoadOnBeam*>  m_loads_on_beams;

        /** @} */

		/** In base classes, process loads on elements and populate the \a m_loads_at_each_dof_equivs and \a m_extra_stress_for_each_element
		  */
		virtual void internalComputeStressAndEquivalentLoads();


	};
}
