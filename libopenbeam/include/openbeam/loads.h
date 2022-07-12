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

#include <openbeam/CElement.h>
#include <openbeam/types.h>

#include <memory>

namespace openbeam
{
/** Base for all loads that can be applied to beam, distributed or applied to a
 * single point. */
struct CLoadOnBeam
{
    CLoadOnBeam()          = default;
    virtual ~CLoadOnBeam() = default;

    using Ptr      = std::shared_ptr<CLoadOnBeam>;
    using ConstPtr = std::shared_ptr<const CLoadOnBeam>;

    virtual void computeStressAndEquivalentLoads(
        const CElement* el, ElementStress& stress,
        std::vector<array6>& loads) = 0;

    /** Class factory from element name, or nullptr for an unknown element:
     *  Element names:
     *		- "TEMPERATURE": CLoadConstTemperature
     *		- "DISTRIB_UNIFORM": CLoadDistributedUniform
     *		- "CONCENTRATED": CLoadConcentratedForce
     *		- "TRIANGULAR": CLoadDistributedTriangular
     */
    static CLoadOnBeam::Ptr createLoadByName(const std::string& sName);

    /** Parse a set of parameters by (casi insensitive) name and set the element
     * values from them. Each element must document the supported parameters and
     * their meaning.
     */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx) = 0;

    /** Decompose the distributed load as needed into the set of elements in
     * which the original element has been meshed */
    virtual void meshLoad(
        CStructureProblem&         meshed_fem,
        const std::vector<size_t>& meshed_element_idxs,
        const size_t               original_bar_idx,
        const CStructureProblem&   original_fem) const = 0;
};

/** A "load" for a constant increase of temperature in the whole element.
 *  Accepted parameters loadable from problem files:
 *   - "deltaT": Increment of temperature (in C degrees)
 */
struct CLoadConstTemperature : public CLoadOnBeam
{
    CLoadConstTemperature(const num_t inc_temp) : m_incr_temp(inc_temp) {}

    CLoadConstTemperature() = default;

    virtual void computeStressAndEquivalentLoads(
        const CElement* el, ElementStress& stress, std::vector<array6>& loads);
    /** See declaration in base class */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx);

    /** Decompose the distributed load as needed into the set of elements in
     * which the original element has been meshed */
    virtual void meshLoad(
        CStructureProblem&         meshed_fem,
        const std::vector<size_t>& meshed_element_idxs,
        const size_t               original_bar_idx,
        const CStructureProblem&   original_fem) const;

    //!< Temperature increment value (in C)
    num_t m_incr_temp = UNINITIALIZED_VALUE;
};

/** Distributed, uniform load over the entire length of a beam with a \a q(N/m)
 * and a director vector. Accepted parameters loadable from problem files:
 *   - "Q": Load density (N/m)
 *   - "DX", "DY" & "DZ": Director vector (must not be unitary)
 */
struct CLoadDistributedUniform : public CLoadOnBeam
{
    CLoadDistributedUniform(num_t q_, num_t vx, num_t vy, num_t vz) : q(q_)
    {
        dir[0] = vx;
        dir[1] = vy;
        dir[2] = vz;
    }

    CLoadDistributedUniform() : q(UNINITIALIZED_VALUE)
    {
        dir[0] = dir[1] = dir[2] = UNINITIALIZED_VALUE;
    }

    virtual void computeStressAndEquivalentLoads(
        const CElement* el, ElementStress& stress, std::vector<array6>& loads);
    /** See declaration in base class */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx);

    /** Decompose the distributed load as needed into the set of elements in
     * which the original element has been meshed */
    virtual void meshLoad(
        CStructureProblem&         meshed_fem,
        const std::vector<size_t>& meshed_element_idxs,
        const size_t               original_bar_idx,
        const CStructureProblem&   original_fem) const;

    num_t q;  //!< Load density (N/m)
    num_t dir[3];  //!< Director vector, in GLOBAL coordinates.
};

/** Distributed, triangular or trapezoidal load over the entire length of a beam
 * with a \a q_ini(N/m), \a q_end(N/m) and a director vector. Accepted
 * parameters loadable from problem files:
 *   - "q_ini": Load density at the start point (N/m)
 *   - "q_end": Load density at the end point (N/m)
 *   - "DX", "DY" & "DZ": Director vector (must not be unitary)
 */
struct CLoadDistributedTriangular : public CLoadOnBeam
{
    CLoadDistributedTriangular(
        num_t q_ini_, num_t q_end_, num_t vx, num_t vy, num_t vz)
        : q_ini(q_ini_), q_end(q_end_)
    {
        dir[0] = vx;
        dir[1] = vy;
        dir[2] = vz;
    }

    CLoadDistributedTriangular()
        : q_ini(UNINITIALIZED_VALUE), q_end(UNINITIALIZED_VALUE)
    {
        dir[0] = dir[1] = dir[2] = UNINITIALIZED_VALUE;
    }

    virtual void computeStressAndEquivalentLoads(
        const CElement* el, ElementStress& stress, std::vector<array6>& loads);
    /** See declaration in base class */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx);

    /** Decompose the distributed load as needed into the set of elements in
     * which the original element has been meshed */
    virtual void meshLoad(
        CStructureProblem&         meshed_fem,
        const std::vector<size_t>& meshed_element_idxs,
        const size_t               original_bar_idx,
        const CStructureProblem&   original_fem) const;

    /// Load density at the start and end points of the beam (N/m)
    num_t q_ini, q_end;
    num_t dir[3] = {0, 0, 0};  //!< Director vector, in GLOBAL coordinates.
};

/** Concentrated force applied at any point along the beam, in an arbitrary
 * direction given by a director vector. Accepted parameters loadable from
 * problem files:
 *   - "P": Load value (N)
 *   - "D": Distance from the first node to the application point (m)
 *   - "DX", "DY" & "DZ": Director vector (must not be unitary)
 */
struct CLoadConcentratedForce : public CLoadOnBeam
{
    /** \param[in] P     Force modulus (N)
     * \param[in] dist  Distance from the first beam node to the point of the
     * force is applied to (m) \param[in] vx,vy,vz  Normalized director vector
     * (in global coordinates) of the force
     */
    CLoadConcentratedForce(num_t P_, num_t dist_, num_t vx, num_t vy, num_t vz)
        : P(P_), dist(dist_)
    {
        dir[0] = vx;
        dir[1] = vy;
        dir[2] = vz;
    }

    CLoadConcentratedForce() : P(UNINITIALIZED_VALUE), dist(UNINITIALIZED_VALUE)
    {
        dir[0] = dir[1] = dir[2] = UNINITIALIZED_VALUE;
    }

    virtual void computeStressAndEquivalentLoads(
        const CElement* el, ElementStress& stress, std::vector<array6>& loads);
    /** See declaration in base class */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx);

    /** Decompose the distributed load as needed into the set of elements in
     * which the original element has been meshed */
    virtual void meshLoad(
        CStructureProblem&         meshed_fem,
        const std::vector<size_t>& meshed_element_idxs,
        const size_t               original_bar_idx,
        const CStructureProblem&   original_fem) const;

    num_t P;  //!< Load modulus (N)
    num_t dist;  //!< Load modulus (N)
    num_t dir[3];  //!< Director vector, in GLOBAL coordinates.
};

}  // namespace openbeam
