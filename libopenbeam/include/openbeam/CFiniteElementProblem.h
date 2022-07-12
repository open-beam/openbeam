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

#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/system/CTimeLogger.h>
#include <openbeam/CElement.h>
#include <openbeam/DrawStructureOptions.h>
#include <openbeam/types.h>

#include <Eigen/Sparse>
#include <cstdint>
#include <fstream>
#include <iostream>

namespace openbeam
{
struct RenderInitData;  // Fwd. decl. (defined in
                        // CFiniteElementProblem::saveAsImage)

/** 0-based indices for DOFs */
enum class DoF_index : uint8_t
{
    DX = 0,
    DY,
    DZ,
    RX,
    RY,
    RZ
};

struct NodeDoF
{
    NodeDoF(size_t _node_id, DoF_index _dof) : nodeId(_node_id), dof(_dof) {}
    NodeDoF(size_t _node_id, uint8_t _dof)
        : nodeId(_node_id), dof(static_cast<DoF_index>(_dof))
    {
    }

    size_t    nodeId;
    DoF_index dof;  //!< In the range [0-5]

    uint8_t dofAsInt() const { return static_cast<uint8_t>(dof); }
};

enum class StaticSolverAlgorithm : uint8_t
{
    LLT = 0,
    SVD
};

/** Extra output information from \a assembleProblem() */
struct BuildProblemInfo
{
    struct TDoFType
    {
        size_t bounded_index;  //!< Index within F_b or U_b, if the DOF is
                               //!< bounded; string::npos otherwise
        size_t free_index;  //!< Index within F_f or U_f, if the DOF is free;
                            //!< string::npos otherwise
    };

    Eigen::SparseMatrix<num_t> K_bb;
    Eigen::SparseMatrix<num_t> K_ff;
    Eigen::SparseMatrix<num_t> K_bf;

    std::vector<size_t>   free_dof_indices;
    std::vector<size_t>   bounded_dof_indices;
    std::vector<TDoFType> dof_types;

    /// A vector of length = "bounded_dof_indices.size()" with all the
    /// constrains, each value for one DoF, in the order as they appear in \a
    /// bounded_dof_indices
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> U_b;

    /// A vector of length = "free_dof_indices.size()" with the overall load at
    /// each free DOF, in the order as they appear in \a free_dof_indices
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> F_f;
};

/** Output information from \a solveStatic() */
struct StaticSolveProblemInfo
{
    /// Information from the assembly of the problem
    BuildProblemInfo build_info;

    /// A vector of length = "build_info.bounded_dof_indices.size()" with all
    /// the reaction forces, DOFs in the same order than \a bounded_dof_indices
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> F_b;

    /// A vector of length = "build_info.free_dof_indices.size()" with the
    /// displacement of free DOFs, in the same order than \a free_dof_indices
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> U_f;

    /// The full F vector (bounded+free DOFs)
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> F;

    /// The full U vector (bounded+free DOFs)
    Eigen::Matrix<num_t, Eigen::Dynamic, 1> U;
};

/** Options and parameters for CStructureProblem::mesh()
 */
struct MeshParams
{
    MeshParams();

    double max_element_length;  //!< In meters (m)
};

/** Results from CStructureProblem::mesh() after meshing a structure into a FEM
 * with smaller elements.
 */
struct MeshOutputInfo
{
    MeshOutputInfo() = default;

    /// The first N nodes in the FEM correspond to the original nodes in the
    /// structure before meshing. This variable holds that "N".
    size_t num_original_nodes = 0;

    /// List of all intermediary nodes (incl the original ones) at which element
    /// [i] is connected.
    std::deque<std::vector<size_t>> element2nodes;

    /// List of smaller element IDs resulting from meshing the element [i]
    std::deque<std::vector<size_t>> element2elements;
};

struct ImageSaveOutputInfo
{
    ImageSaveOutputInfo() = default;

    unsigned int img_width = 0, img_height = 0;
};

/** The options of the static solver \a solveStatic() */
struct StaticSolverOptions
{
    StaticSolverOptions() = default;

    StaticSolverAlgorithm algorithm{StaticSolverAlgorithm::LLT};
    bool                  nonLinearIterative = false;
};

struct StressInfo
{
    StressInfo() {}

    /** The i'th entry is another vector with the stress of all the faces of
     * the i'th element. Typically for 1D elements (beams, springs) each
     * entry will only contain two stress values, one for each end of the
     * element.
     */
    std::vector<ElementStress> element_stress;
};

/** A complete problem of finite elements.
 *  It contains:
 *     - Elements: A list of finite elements from which the problem is built of.
 *     - Nodes: A list of 6 DoF nodes at certain space locations. Elements are
 * always defined between 2 or more nodes.
 *     - Constraints: External loads and DoF movement restrictions. They are
 * applied over the nodes.
 *
 * \sa CFiniteElementProblem
 */
class CFiniteElementProblem
{
   public:
    using constraint_list_t = std::map<size_t, num_t>;
    using load_list_t       = std::map<size_t, num_t>;

    CFiniteElementProblem() = default;
    virtual ~CFiniteElementProblem();

    // ----------------------------------------------------------------------------
    /** @name High-level management
        @{ */

    virtual void clear();  //!< Delete all elements, nodes and constraints in
                           //!< this problem, completely emptying it.

    /** Discard the current problem and loads it by parsing the given text input
     * stream. The expected format of the file is described in XXXX.
     *
     *  \param[in] is        The input stream.
     *  \param[out] errMsg   The error messages found while parsing the text.
     *  \param[out] warnMsg  The warning messages found while parsing the text.
     *
     *  \return true on Success
     */
    bool loadFromStream(
        std::istream&                              is,
        const mrpt::optional_ref<vector_string_t>& errMsg  = std::nullopt,
        const mrpt::optional_ref<vector_string_t>& warnMsg = std::nullopt);

    /** Discard the current problem and loads it by parsing the given text file.
     *  The expected format of the file is described in XXXX.
     *
     *  \param[in] is        The input stream.
     *  \param[out] errMsg   The error messages found while parsing the text.
     *  \param[out] warnMsg  The warning messages found while parsing the text.
     *
     *  \return true on Success
     */
    bool loadFromFile(
        const std::string&                         file,
        const mrpt::optional_ref<vector_string_t>& errMsg  = std::nullopt,
        const mrpt::optional_ref<vector_string_t>& warnMsg = std::nullopt);

    /** Saves a representation of the problem to a SVG file.
     * \param options Many parameters and switches to control what will be drawn
     * and hiden. \return true on success
     */
    bool saveAsImageSVG(
        const std::string& file, const DrawStructureOptions& options,
        const StaticSolveProblemInfo* solver_info  = nullptr,
        const MeshOutputInfo*         meshing_info = nullptr,
        ImageSaveOutputInfo*          out_img_info = nullptr) const;
    bool saveAsImagePNG(
        const std::string& file, const DrawStructureOptions& options,
        const StaticSolveProblemInfo* solver_info  = nullptr,
        const MeshOutputInfo*         meshing_info = nullptr,
        ImageSaveOutputInfo*          out_img_info = nullptr) const;
    bool saveAsImage(
        const std::string& file, const bool is_svg,
        const DrawStructureOptions&   options,
        const StaticSolveProblemInfo* solver_info  = nullptr,
        const MeshOutputInfo*         meshing_info = nullptr,
        ImageSaveOutputInfo*          out_img_info = nullptr) const;

    bool renderToCairoContext(
        void* _cairo_context, const RenderInitData& ri,
        const DrawStructureOptions&   options,
        const StaticSolveProblemInfo* solver_info,
        const MeshOutputInfo*         meshing_info) const;

    /**    @} */
    // ----------------------------------------------------------------------------
    /** @name Problem elements
        @{ */

    /** Insert a new element in the problem
     *  \return The element index */
    size_t insertElement(CElement::Ptr el);

    /** Insert a new element in the problem
     *  \return The element index */
    template <typename ElementClass, typename... _Args>
    size_t createElement(_Args&&... __args)
    {
        return insertElement(
            std::make_shared<ElementClass>(std::forward<_Args>(__args)...));
    }

    /** Return a constant pointer to the i'th element in the problem. */
    CElement::ConstPtr getElement(size_t i) const;

    /** Return a pointer to the i'th element in the problem. */
    const CElement::Ptr& getElement(size_t i);

    /** Get the number of elements in the problem */
    size_t getNumberOfElements() const { return m_elements.size(); }

    /** @} */
    // ----------------------------------------------------------------------------
    /** @name Problem constraints
        @{ */

    /** Adds a new displacement constraint to the problem, at the DoF \a
     * dof_index (indices wrt list returned by \a getProblemDoFs() ).
     */
    void insertConstraint(const size_t dof_index, const num_t value = 0);

    const constraint_list_t& getAllConstraints() const
    {
        return m_DoF_constraints;
    }

    /** Sets the load (force or moment) applied to a given DOF  \sa addLoadAtDOF
     */
    void setLoadAtDOF(const size_t dof_index, const num_t f);

    /** Adds a load (force or moment) to a given DOF, accumulating to previously
     * existing values if any.  \sa setLoadAtDOF */
    void addLoadAtDOF(const size_t dof_index, const num_t f);

    const load_list_t& getOverallLoadsOnDOFs() const
    {
        return m_loads_at_each_dof;
    }

    /** @} */
    // ----------------------------------------------------------------------------
    /** @name Problem nodes and their reference 3D pose (to this, one must add
       the displacement obtained as solution to the problem).
        @{ */

    /** Set the number of nodes in the problem */
    void setNumberOfNodes(size_t N);

    /** Number of DoF's? */
    size_t getNumberOfNodes() const { return m_node_poses.size(); }

    /** Insert a new node to the problem with the given 6D pose, in global
     * coordinates. \return The index of this new node */
    size_t insertNode(const TRotationTrans3D& p);

    /** Set the reference 6D pose of a node, in global coordinates */
    void setNodePose(size_t idx, const TRotationTrans3D& p);

    /** Set the reference 3D pose of a node, in global coordinates */
    void setNodePose(size_t idx, const num_t x, const num_t y, const num_t z);

    /** The 3D location of a node; its orientation (if different than global
     * coordinates) serves as a local frame for constraints. */
    TRotationTrans3D& getNodePose(size_t i)
    {
        ASSERTDEB_(i < m_node_poses.size());
        return m_node_poses[i];
    }
    /** The 3D location of a node; its orientation (if different than global
     * coordinates) serves as a local frame for constraints. */
    const TRotationTrans3D& getNodePose(size_t i) const
    {
        ASSERTDEB_(i < m_node_poses.size());
        return m_node_poses[i];
    }

    /** Returns the 3D location of the final deformed state after solving the
     * problem */
    void getNodeDeformedPosition(
        size_t i, Vector3& out_final_point,
        const StaticSolveProblemInfo& solver_info,
        const num_t                   exageration_factor = 1) const;

    /** Returns the maximum absolute value translation in any X,Y,Z directions
     * for a static problem solution */
    num_t getMaximumDeformedDisplacement(
        const StaticSolveProblemInfo& solver_info) const;

    /** Computes the bounding box of all existing nodes */
    void getBoundingBox(
        num_t& min_x, num_t& max_x, num_t& min_y, num_t& max_y,
        bool                          deformed              = false,
        const StaticSolveProblemInfo* solver_info           = nullptr,
        num_t                         deformed_scale_factor = 1.0) const;

    /** @} */
    // ----------------------------------------------------------------------------
    /** @name Problem DOFs (abstract list of DoF, doesn't have quantitative
       displacement values)
        @{ */

    /** Return the list of all DoF in the problem . \sa
     * getProblemDoFsDescription */
    const std::vector<NodeDoF>& getProblemDoFs()
    {
        updateListDoFs();
        return m_problem_DoFs;
    }

    /** Make a list with all DoF in the problem sorted by nodes. \sa
     * getProblemDoFs */
    std::string getProblemDoFsDescription();

    /** Return the complementary list of DOFs, such as "return \CUP ds = 1:N"
     * and "return \CAP ds = \empty".
     */
    static std::vector<size_t> complementaryDoFs(
        const std::vector<size_t>& ds, const size_t nTotalDOFs);

    /** Returns the 0-based index of the n'th DOF (0-5) of node "nNode" in the
     * list of DOFs to consider in the problem. Callable only after \a
     * buildAll(). Returns string::npos if the DOF is NOT considered in the
     * problem.
     */
    size_t getDOFIndex(const size_t nNode, const DoF_index n) const;

    /** @} */
    // ----------------------------------------------------------------------------
    /** @name Problem solving
        @{ */

    /** Update all internal lists after changing the problem.
     * This actually calls:
     *  - 	updateElementsOrientation()
     *  - 	updateListDoFs()
     */
    virtual void updateAll();

    /** For any partitioning of the DoFs into fixed (restricted, boundary
     * conditions) and the rest (the free variables \a free_dof_indices), where
     * DoF indices refer to \a m_problem_DoFs, computes three sparse matrices:
     * K_{FF} (free-free), K_{BB} (boundary-boundary) and K_{BF} (boundary-free)
     * which define the stiffness matrix of the problem. \note The free vs
     * constrained DoF's are automatically determined from \a m_DoF_constraints
     * \sa solveStatic
     */
    void assembleProblem(BuildProblemInfo& out_info);

    /** Solve the static FE problem, returning the resulting displacements and
     * reaction forces.
     *
     * \note This method internally calls \a assembleProblem()
     *
     * \sa assembleProblem, postProcCalcStress
     */
    void solveStatic(
        StaticSolveProblemInfo&    out_info,
        const StaticSolverOptions& opts = StaticSolverOptions());

    /** After solving the problem with \a solveStatic, you can optionally call
     * this post-processing method to evaluate the stress of all the elements.
     *
     * \sa solveStatic
     */
    void postProcCalcStress(
        StressInfo& out_stress, const StaticSolveProblemInfo& solver_info);

    std::string getNodeLabel(
        const size_t idx) const;  //!< "N%i" or custom label

    /** @} */
    // ----------------------------------------------------------------------------
   protected:
    /** @name Main data
        @{ */

    std::deque<TRotationTrans3D> m_node_poses;
    /// empty: default, custom label otherwise
    std::vector<std::string>  m_node_labels;
    std::deque<CElement::Ptr> m_elements;

    /** List of constrainsts for each "fixed/constrained" DoF.
     *  Map key are indices in \a m_problem_DoFs.
     *  Map values are displacement in that DoF wrt the current node pose. Units
     * are SI (meters or radians). \sa getProblemDoFs
     */
    constraint_list_t m_DoF_constraints;

    /**  The vector of overall external loads (F_L) - Map keys are indices of \a
     * m_problem_DoFs  */
    load_list_t m_loads_at_each_dof;

    /**  The vector of overall loads (F_L') on each DoF due to distributed
     * forces - Map keys are indices of \a m_problem_DoFs Updated by \a
     * assembleProblem() via \a internalComputeStressAndEquivalentLoads()
     */
    load_list_t m_loads_at_each_dof_equivs;

    /**  The extra stress to add up to each element as computed by \a
     * internalComputeStressAndEquivalentLoads map: element index -> vector for
     * each "port" -> vector of 6 stresses values.
     */
    std::map<size_t, ElementStress> m_extra_stress_for_each_element;

    /** @} */

    /** In base classes, process loads on elements and populate the \a
     * m_loads_at_each_dof_equivs and \a m_extra_stress_for_each_element
     */
    virtual void internalComputeStressAndEquivalentLoads() {}

    /** Internal common implementation of loadFrom*() methods() */
    bool internal_loadFromYaml(
        const mrpt::containers::yaml&              f,
        const mrpt::optional_ref<vector_string_t>& errMsg,
        const mrpt::optional_ref<vector_string_t>& warnMsg);

    void internal_parser1_Parameters(
        const mrpt::containers::yaml& f, EvaluationContext& ctx) const;

    /** From the list of elements and their properties and connections, build
     * the list of DoFs relevant to the problem.
     */
    void updateListDoFs();

    /** From the list of elements, update m_node_connections */
    void updateNodeConnections();

    /** Call the "updateOrientationFromNodePositions" in each element */
    void updateElementsOrientation();

    /** Used in \a m_problem_DoFs_inverse_list  */
    struct TProblemDOFIndicesForNode
    {
        TProblemDOFIndicesForNode() = default;

        /// Index of that DOF into \a m_problem_DoFs, or -1 if DOF not
        /// considered in the problem.
        std::array<int, 6> dof_index = {-1, -1, -1, -1, -1, -1};
    };

    /** The list of DoFs variables of my problem.
     *  Built by \a updateListDoFs()
     *  \sa m_problem_DoFs_inverse_list, m_problem_DoF_type
     */
    std::vector<NodeDoF> m_problem_DoFs;

    /** Built together with m_problem_DoFs by \a updateListDoFs , holds the
     * mapping from node index -> 6 numbers, with the index of that global DOF
     * in the list of problem DOFs, or -1 if the DOF is not considered.
     */
    std::vector<TProblemDOFIndicesForNode> m_problem_DoFs_inverse_list;

    struct TNodeElementConnection
    {
        unsigned char element_face_id;  //!< To which face in that element
        used_DoFs_t   dofs;  //!< DoFs used by that face in that element
    };

    typedef std::map<size_t, TNodeElementConnection>
        TNodeConnections;  //!< For each node, this map has the list of
                           //!< connected elements and the type of the
                           //!< connection.

    /**  A vector with an item for each node (list in \a m_node_poses)
     *  Each item is a map from element_id -> TNodeElementConnection
     */
    std::vector<TNodeConnections> m_node_connections;
};
}  // namespace openbeam
