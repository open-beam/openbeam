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
#include <openbeam/CStructureProblem.h>
#include <openbeam/elements.h>
#include <openbeam/loads.h>

using namespace std;
using namespace openbeam;

void CLoadConcentratedForce::computeStressAndEquivalentLoads(
    const CElement* el, ElementStress& stress, std::vector<array6>& loads)
{
    // make sure director vector is unitary:
    ASSERT_(
        std::abs(1 - (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2])) <
        1e-6);

    const TRotationTrans3D& node0 =
        el->getParent()->getNodePose(el->conected_nodes_ids[0]);
    const TRotationTrans3D& node1 =
        el->getParent()->getNodePose(el->conected_nodes_ids[1]);

    num_t Ax = node1.t.x - node0.t.x;
    num_t Ay = node1.t.y - node0.t.y;
    num_t Az = node1.t.z - node0.t.z;

    const num_t L2 = square(Ax) + square(Ay) + square(Az);
    const num_t L  = std::sqrt(L2);
    ASSERT_(L > 0);
    const num_t _1_L = 1 / L;

    // Normalize direction vector:
    Ax *= _1_L;
    Ay *= _1_L;
    Az *= _1_L;

    OB_TODO("Stress: general 3D case")
    // TODO: Make this real for 3D: reaction should be perp.
    //  to the beam in the direction defined by the plane "beam<->load dir".

    // Cross product: A x dir
    const num_t cross_prod[3] = {
        Ay * dir[2] - Az * dir[1], -Ax * dir[2] + Az * dir[0],
        Ax * dir[1] - Ay * dir[0]};

    // Split "q" into perpendicular & tangent load densities:
    const num_t proy_long = (Ax * dir[0] + Ay * dir[1] + Az * dir[2]);
    const num_t proy_perp =
        std::sqrt(1 - proy_long * proy_long) * (cross_prod[2] < 0 ? +1. : -1.);

    const num_t P_t = P * proy_long;
    const num_t P_n = P * proy_perp;

    if (std::abs(proy_long) > 1e-5)
        throw std::runtime_error("Only perpendicular forces supported by now!");

    const num_t a = this->dist;
    const num_t b = L - a;

    if (a < 0 || b < 0)
        throw std::runtime_error(
            "Error: Point of application is out of the beam!");

    const num_t a2 = a * a;
    const num_t b2 = b * b;
    const num_t L3 = L * L2;

    // Stress -------------------------------------------

    // Stress components, in the element local coordinates:
    Eigen::Matrix<num_t, 3, 1> f1, f2;
    f1.setZero();
    f2.setZero();

    num_t m1, m2;

    if (dynamic_cast<const CElementBeam_2D_RR*>(el) != nullptr)
    {
        const CElementBeam_2D_RR* e =
            dynamic_cast<const CElementBeam_2D_RR*>(el);

        f1[0] = 0;
        f1[1] = P_n * (b2 / L3) * (3 * a + b);
        m1    = P_n * a * b2 / L2;
        f2[0] = 0;
        f2[1] = P_n * (a2 / L3) * (a + 3 * b);
        m2    = -P_n * a2 * b / L2;
    }
    else
        throw std::runtime_error(
            "Unsupported element type for load 'CLoadConstTemperature'");

#if 0
	cout << "f1: " << f1.transpose() << "<br>" <<endl;
	cout << "f2: " << f2.transpose() << "<br>" <<endl;
#endif

    // Stress:
    stress.resize(2);

    stress[0].N  = -f1[0];
    stress[0].Vy = -f1[1];
    stress[0].Mz = -m1;

    stress[1].N  = f2[0];
    stress[1].Vy = f2[1];
    stress[1].Mz = m2;

    // Equivalent load -------------------------------------------
    const Matrix33& R =
        el->getGlobalOrientation()
            .getRot();  // Reference to the 3D rotation matrix (3x3) of the beam

    const Eigen::Matrix<num_t, 3, 1> F1 =
        -R * f1;  // "-" since loads are the inverse of reactions/stress.
    const Eigen::Matrix<num_t, 3, 1> F2 = -R * f2;

    loads.resize(2);

    loads[0][0] = F1[0];
    loads[0][1] = F1[1];
    loads[0][2] = F1[2];
    loads[0][3] = 0;
    loads[0][4] = 0;
    loads[0][5] = -m1;

    loads[1][0] = F2[0];
    loads[1][1] = F2[1];
    loads[1][2] = F2[2];
    loads[1][3] = 0;
    loads[1][4] = 0;
    loads[1][5] = -m2;
}

/** Parse a set of parameters by (casi insensitive) name and set the element
 * values from them. */
void CLoadConcentratedForce::loadParamsFromSet(
    const mrpt::containers::yaml& p, const EvaluationContext& ctx)
{
    this->P    = ctx.evaluate(p["p"]);
    this->dist = ctx.evaluate(p["dist"]);
    dir[0]     = ctx.evaluate(p["DX"]);
    dir[1]     = ctx.evaluate(p["DY"]);
    if (p.has("DZ")) dir[2] = ctx.evaluate(p["DZ"]);
}

/** Decompose the distributed load as needed into the set of elements in which
 * the original element has been meshed */
void CLoadConcentratedForce::meshLoad(
    CStructureProblem&         meshed_fem,
    const std::vector<size_t>& meshed_element_idxs,
    const size_t original_bar_idx, const CStructureProblem& original_fem) const
{
    // Find out the original length of the bar:
    const size_t node0 = meshed_fem.getElement(*meshed_element_idxs.begin())
                             ->conected_nodes_ids[0];
    const size_t node1 = meshed_fem.getElement(*meshed_element_idxs.rbegin())
                             ->conected_nodes_ids[1];
    const double org_bar_length =
        meshed_fem.getNodePose(node0).distanceTo(meshed_fem.getNodePose(node1));

    const size_t nE = meshed_element_idxs.size();
    double       Al = org_bar_length / nE;

    // Create the new load:
    const size_t idx_affected_element =
        std::min(floor(this->dist / Al), static_cast<double>(nE - 1));

    auto new_load = std::make_shared<CLoadConcentratedForce>(*this);
    new_load->dist -= idx_affected_element * Al;

    meshed_fem.addLoadAtBeam(idx_affected_element, new_load);
}

mrpt::opengl::CSetOfObjects::Ptr CLoadConcentratedForce::getVisualization(
    const CFiniteElementProblem& fem, const DrawStructureOptions& options,
    const DrawElementExtraParams& draw_el_params,
    const MeshOutputInfo*         meshing_info) const
{
    return {};
}
