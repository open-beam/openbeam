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

#include "internals.h"

using namespace std;
using namespace openbeam;

/**
 */
void openbeam::internal::computeOrientationFromTwoPoints(
    const TPoint3D& t1, const TPoint3D& t2, const num_t rotation_around_x,
    TRotation3D& out_R)
{
    // Vector: 1 -> 2
    const TPoint3D p12 = TPoint3D(
        t2.coords[0] - t1.coords[0], t2.coords[1] - t1.coords[1],
        t2.coords[2] - t1.coords[2]);
    const num_t len = p12.norm();

    TMatrix33 R;
    if (len > 0)
    {
        // Normal axis:
        const num_t len_inv = 1 / len;

        num_t X[3], Y[3],
            Z[3];  // Coordinates of the NEW axes wrt GLOBAL coordinates frame

        // New X:
        X[0] = p12.coords[0] * len_inv;
        X[1] = p12.coords[1] * len_inv;
        X[2] = p12.coords[2] * len_inv;

        // New Z: orthogonal to X and with Zy=0:
        if (X[2] == 0)
        {
            Z[0] = 0;
            Z[1] = 0;
            Z[2] = 1;
        }
        else
        {
            if (X[0] != 0 || X[1] != 0)
            {
                Z[0] = -X[0];
                Z[1] = -X[1];
                Z[2] = (X[0] * X[0] + X[1] * X[1]) / X[2];
                const num_t norm_fact =
                    num_t(1) /
                    std::sqrt(square(Z[0]) + square(Z[1]) + square(Z[2]));
                Z[0] *= norm_fact;
                Z[1] *= norm_fact;
                Z[2] *= norm_fact;
            }
            else
            {
                Z[0] = -1;
                Z[1] = 0;
                Z[2] = 0;
            }
        }

        // New Y: cross product:
        Y[0] = Z[1] * X[2] - Z[2] * X[1];
        Y[1] = Z[2] * X[0] - Z[0] * X[2];
        Y[2] = Z[0] * X[1] - Z[1] * X[0];

        // Now, rotate around the new X axis
        // "m_design_rotation_around_linear_axis" radians
        R(0, 0) = X[0];
        R(0, 1) = Y[0];
        R(0, 2) = Z[0];  // X' is not affected.
        R(1, 0) = X[1];
        R(1, 1) = Y[1];
        R(1, 2) = Z[1];
        R(2, 0) = X[2];
        R(2, 1) = Y[2];
        R(2, 2) = Z[2];
    }
    else
    {
        // For elements without any length, use global coordinates:
        R.setIdentity();
    }

    if (rotation_around_x != 0)
    {
        // Y'' = sin()*Z' + cos()*Y'
        // Z'' = cos()*Z' - sin()*Y'
        //
        Eigen::Matrix<num_t, 2, 2> axis_rot;
        const num_t                c = cos(rotation_around_x);
        const num_t                s = sin(rotation_around_x);
        axis_rot(0, 0)               = c;
        axis_rot(0, 1)               = -s;
        axis_rot(1, 0)               = s;
        axis_rot(1, 1)               = c;

        R.block(1, 0, 2, 3) = axis_rot * R.block(1, 0, 2, 3);
    }

    out_R.setRot(R);
}
