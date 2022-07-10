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

#include <openbeam/types.h>

#if OPENBEAM_HAS_CAIRO
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include <cairommconfig.h>
#endif

namespace openbeam
{
namespace internal
{
/**
 */
template <class CAIROCR>
void drawLocalScaledSegments(
    CAIROCR& cr, const TRotationTrans3D& node_pose, const double scale,
    const std::vector<TPoint3D>& seq_points_local)
{
    const Matrix33& node_rot = node_pose.r.getRot();

    // then rotate to the nodal coordinates (typ. coincides with global coords)
    const size_t          nPts = seq_points_local.size();
    std::vector<TPoint3D> seq_points_global;
    if (node_pose.r.isIdentity())
    {
        seq_points_global = seq_points_local;
        for (size_t k = 0; k < nPts; k++)
            // POINT_GLOBAL = NODE_GLOBAL + MATRIX33 * POINT_LOCAL
            if (scale == 1)
                for (int l = 0; l < 3; l++)
                    seq_points_global[k].coords[l] += node_pose.t.coords[l];
            else
                for (int l = 0; l < 3; l++)
                    seq_points_global[k].coords[l] =
                        node_pose.t.coords[l] +
                        scale * seq_points_global[k].coords[l];
    }
    else
    {
        seq_points_global.resize(nPts);
        for (size_t k = 0; k < nPts; k++)
        {
            // POINT_GLOBAL = NODE_GLOBAL + MATRIX33 * POINT_LOCAL
            for (int l = 0; l < 3; l++)
                seq_points_global[k].coords[l] =
                    node_pose.t.coords[l] +
                    scale *
                        (node_rot.coeff(l, 0) * seq_points_local[k].coords[0] +
                         node_rot.coeff(l, 1) * seq_points_local[k].coords[1] +
                         node_rot.coeff(l, 2) * seq_points_local[k].coords[2]);
        }
    }

    // Draw the lines:
    for (size_t k = 0; k < nPts; k++)
    {
        const double xx = seq_points_global[k].coords[0];
        const double yy = seq_points_global[k].coords[1];
        if (k == 0)
            cr->move_to(xx, yy);
        else
            cr->line_to(xx, yy);
    }
    cr->stroke();

}  // end of drawLocalScaledSegments

/** */
void computeOrientationFromTwoPoints(
    const TPoint3D& t1, const TPoint3D& t2, const num_t rotation_around_x,
    TRotation3D& R);

}  // namespace internal
}  // namespace openbeam
