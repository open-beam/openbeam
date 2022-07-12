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

#include <openbeam/DrawStructureOptions.h>
#include <openbeam/types.h>

#include <memory>

#if OPENBEAM_HAS_QT5Svg
#include <QtSvg>
#endif

namespace openbeam
{
class CFiniteElementProblem;
struct MeshParams;
struct MeshOutputInfo;
class CStructureProblem;

struct TStiffnessSubmatrix
{
    size_t   edge_in, edge_out;
    Matrix66 matrix;
};

/// True/false for each DoF: x y z yaw pitch roll
using used_DoFs_t = std::array<bool, 6>;

/** Virtual base for any finite element.
 */
class CElement
{
   public:
    using Ptr      = std::shared_ptr<CElement>;
    using ConstPtr = std::shared_ptr<const CElement>;

    /** Class factory from element name, or nullptr for an unknown element:
     *  Element names:
     *		- "BEAM2D_AA": CElementBeam_2D_AA
     *		- "BEAM2D_AR": CElementBeam_2D_AR
     *		- "BEAM2D_RA": CElementBeam_2D_RA
     *		- "BEAM2D_RR": CElementBeam_2D_RR
     *		- "SPRING_1D": CElementSpring
     *		- "SPRING_TORSION": CElementTorsionSpring
     */
    static Ptr createElementByName(const std::string& sName);

    /** The list of connected nodes at each "port" or "face"
     * It's set to the correct size at constructions automatically.
     */
    std::vector<size_t> conected_nodes_ids;

    /** Must return the number of "ports" or "edges" of this element. This is
     * the same than the length of \a conected_nodes_ids and the vectors
     * returned by \a getLocalStiffnessMatrices  */
    unsigned char getNumberEdges() const { return m_nEdges; }

    /** Return the stiffness submatrices (for element local coordinates) between
     * each pair of edges in this element, for the current element state. From
     * all (in,out) edge pairs, only return those with in<=out (the other half
     * is the corresponding transpose).
     */
    virtual void getLocalStiffnessMatrices(
        std::vector<TStiffnessSubmatrix>& outSubMats) const = 0;

    /** Must return which DoFs are used in the returned Stiffness matrices */
    virtual void getLocalDoFs(std::vector<used_DoFs_t>& dofs) const = 0;

    /** Return the local DoFs transformed with the current element pose in \a
     * m_global_orientation */
    void getGlobalDoFs(std::vector<used_DoFs_t>& dofs) const;

    /** This method first calls getLocalStiffnessMatrices then rotates each
     * matrix according to the current element orientation Can be redefined in
     * derived classes for efficiency
     */
    virtual void getGlobalStiffnessMatrices(
        std::vector<TStiffnessSubmatrix>& outSubMats) const;

    inline void setGlobalOrientation(const TRotation3D& new_orientation)
    {
        m_global_orientation = new_orientation;
    }
    inline const TRotation3D& getGlobalOrientation() const
    {
        return m_global_orientation;
    }

    /**  Updates \a m_global_orientation  from the global poses of \a
     * conected_nodes_ids and the value \a m_design_rotation_around_linear_axis
     */
    virtual void updateOrientationFromNodePositions();

    /** For linear elements from i->j, there's a rotation which is undefined if
     * they're fixed between any two points. This parameter model that rotation.
     * It's used during \a updateOrientationFromNodePositions
     */
    void setDesignRotationAroundLinearAxis(num_t ang)
    {
        m_design_rotation_around_linear_axis = ang;
    }

    void setParent(CFiniteElementProblem* parent) { m_parent = parent; }

    /** Parse a set of parameters by (casi insensitive) name and set the element
     * values from them. Each element must document the supported parameters and
     * their meaning.
     */
    virtual void loadParamsFromSet(
        const mrpt::containers::yaml& p, const EvaluationContext& ctx) = 0;

    /** \overload */
    void loadParamsFromSet(const mrpt::containers::yaml& p)
    {
        EvaluationContext def;
        loadParamsFromSet(p, def);
    }

    /** Get the parent FEM problem object */
    CFiniteElementProblem* getParent() const { return m_parent; }

    /** Draws the element to a SVG Cairo context (a pointer to a
     * Cairo::RefPtr<Cairo::Context> casted to void*), according to the passed
     * options */
    virtual void drawSVG(
        void* _cairo_context, const DrawStructureOptions& options,
        const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo* meshing_info) const = 0;
#if OPENBEAM_HAS_QT5Svg
    virtual void drawQtSVG(
        QSvgGenerator& svg, const DrawStructureOptions& options,
        const RenderInitData& ri, const DrawElementExtraParams& draw_el_params,
        const MeshOutputInfo* meshing_info) const = 0;
#endif

    /** Mesh this element into a set of (possibly) smaller ones */
    virtual void do_mesh(
        const size_t my_idx, CStructureProblem& out_fem,
        MeshOutputInfo& out_info, const MeshParams& params) = 0;

   protected:
    CElement(unsigned char nEdges);
    CElement(
        unsigned char nEdges, const size_t from_node_id,
        const size_t to_node_id);  //!< Special case that initializes the first
                                   //!< two \a conected_nodes_ids

    const unsigned char    m_nEdges;
    CFiniteElementProblem* m_parent;

    /// For linear elements, the rotation around +X(local)
    num_t m_design_rotation_around_linear_axis;

    /// Orientation of the element wrt global coords
    TRotation3D m_global_orientation;
};
}  // namespace openbeam
