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

#include <openbeam/config.h>
//
#include <mrpt/core/exceptions.h>  // ASSERT_(), etc.
#include <mrpt/core/format.h>  // mrpt::format()
#include <mrpt/system/CTimeLogger.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

/** Usage: OB_MESSAGE(num) << "blah" << x << endl;
 *  Where num is the minimum OpenBeam verbose level for the message to actually
 * be emitted to std::cout. \sa openbeam::setVerbosityLevel
 */
#define OB_MESSAGE(VERBOSE_LEVEL)                       \
    if (openbeam::getVerbosityLevel() >= VERBOSE_LEVEL) \
    std::cout << "[" << __CURRENT_FUNCTION_NAME__ << "] "

#define OB_TODO(x__) MRPT_TODO(x__)

namespace openbeam
{
inline num_t DEG2RAD(const num_t x) { return x * M_PI / num_t(180.0); }
inline num_t RAD2DEG(const num_t x) { return x * num_t(180.0) / M_PI; }

constexpr num_t UNINITIALIZED_VALUE = (std::numeric_limits<num_t>::max());

template <typename T>
inline T square(T x)
{
    return x * x;
}

extern mrpt::system::CTimeLogger timelog;  //!< A global timelogger for openbeam

/** @name Types
    @{ */

using DynMatrix = Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix66  = Eigen::Matrix<num_t, 6, 6>;
using Matrix33  = Eigen::Matrix<num_t, 3, 3>;
using Vector6   = Eigen::Matrix<num_t, 6, 1>;
using Vector3   = Eigen::Matrix<num_t, 3, 1>;

//!< A set of parameters, indexed by name (case insensitive)
using param_set_t      = std::map<std::string, std::string>;
using map_string2num_t = std::map<std::string, num_t>;
using vector_string_t  = std::vector<std::string>;

/** Stress tensor for a 3D element */
struct FaceStress
{
    FaceStress()  = default;
    ~FaceStress() = default;

    num_t N  = 0;  //!< Axial
    num_t Vy = 0;  //!< Shear Y
    num_t Vz = 0;  //!< Shear Z
    num_t Mx = 0;  //!< Moment X
    num_t My = 0;  //!< Moment Y
    num_t Mz = 0;  //!< Moment Z

    FaceStress& operator+=(const FaceStress& o)
    {
        N += o.N;
        Vy += o.Vy;
        Vz += o.Vz;
        Mx += o.Mx;
        My += o.My;
        Mz += o.Mz;
        return *this;
    }

    template <class MAT>
    FaceStress& operator+=(const Eigen::MatrixBase<MAT>& o)
    {
        ASSERTDEB_(o.size() == 6)
        N += o[0];
        Vy += o[1];
        Vz += o[2];
        Mx += o[3];
        My += o[4];
        Mz += o[5];
        return *this;
    }
};

//!< One entry per element "face".
using ElementStress = std::vector<FaceStress>;

struct EvaluationContext
{
    EvaluationContext() = default;

    /** Return false only if there was an error if there's not "errMsg".
     * Should only process the case of returning true and do nothing else on
     * return false to handle the error. */
    bool parser_evaluate_expression(
        const std::string& sVarVal, num_t& val) const;

    map_string2num_t user_vars;
    vector_string_t* err_msgs  = nullptr;
    vector_string_t* warn_msgs = nullptr;
    unsigned int     lin_num   = 0;
    std::string*     lin       = nullptr;
};

struct TRotation3D
{
    inline TRotation3D() : rot(Matrix33::Identity()), m_is_pure_identity(true)
    {
    }
    TRotation3D(const num_t ang_x, const num_t ang_y, const num_t ang_z);

    inline const Matrix33& getRot() const { return rot; }
    inline void            setRot(const Matrix33& r)
    {
        rot                = r;
        m_is_pure_identity = false;
    }

    static void matrix2angles(
        const Matrix33& R, num_t& ang_x, num_t& ang_y, num_t& ang_z);

    bool isIdentity()
        const;  //!< Test for whether this 3x3 matrix is EXACTLY the identity

   private:
    Matrix33 rot;
    bool     m_is_pure_identity;
};

struct TPoint3D
{
    num_t coords[3];

    TPoint3D() {}
    TPoint3D(const num_t x, const num_t y, const num_t z)
    {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    }
    inline num_t norm_squared() const
    {
        return coords[0] * coords[0] + coords[1] * coords[1] +
               coords[2] * coords[2];
    }
    inline num_t norm() const { return std::sqrt(norm_squared()); }
};

struct TRotationTrans3D
{
    inline TRotationTrans3D() : t(0, 0, 0), r() {}
    inline TRotationTrans3D(
        num_t x, num_t y, num_t z, num_t ang_x, num_t ang_y, num_t ang_z)
        : t(x, y, z), r(ang_x, ang_y, ang_z)
    {
    }

    TPoint3D    t;  //!< Translation vector in 3D
    TRotation3D r;  //!< Rotation in 3D

    inline num_t distanceTo(const TRotationTrans3D& o) const
    {
        return std::sqrt(
            openbeam::square(t.coords[0] - o.t.coords[0]) +
            openbeam::square(t.coords[1] - o.t.coords[1]) +
            openbeam::square(t.coords[2] - o.t.coords[2]));
    }
};

/** @} */

/** Changes the overall library verbosity level: 0=quiet, 1=info, 2=verbose.
 *  \sa getVerbosityLevel
 */
void setVerbosityLevel(int verbose_level);

/** Get the current verbosity level of the library.
 * \sa setVerbosityLevel
 */
int getVerbosityLevel();

/** Return true if the two strings are equal (case insensitive)  */
bool strCmpI(const std::string& s1, const std::string& s2);

/** Returns the numeric representation of a string, or raises an exception if
 * it's not a valid number */
num_t str2num(const std::string& s);

using mrpt::format;

using array6 = std::array<num_t, 6>;

}  // namespace openbeam
