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

#include "CBaseElementBeam.h"

namespace openbeam
{
/** A **2D** beam element whose two ends are "rigid" links (x,y,phi).
 */
class CElementBeam_2D_RR : public CBaseElementBeam
{
   public:
    CElementBeam_2D_RR();
    CElementBeam_2D_RR(const size_t from_node_id, const size_t to_node_id);

    /** Return the stiffness submatrices between each pair of edges in this
     * element, for the current element state.
     */
    virtual void getLocalStiffnessMatrices(
        std::vector<TStiffnessSubmatrix>& outSubMats) const;

    virtual void getLocalDoFs(std::vector<used_DoFs_t>& dofs) const;
};
}  // namespace openbeam
