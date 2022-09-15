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

#include <mrpt/containers/printf_vector.h>
#include <mrpt/core/round.h>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/system/string_utils.h>
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>

#include "ExpressionEvaluator.h"

using namespace std;
using namespace openbeam;
using namespace Eigen;

#if 0
/** Used to parse "k1=v1,k2=v2,a,b,c" with \a parseParams() */
struct TParsedParams
{
    bool hasKey(const std::string& key) const
    {
        return key_vals.end() != key_vals.find(key);
    }

    mrpt::containers::yaml key_vals;
    vector_string_t        rest_values;
};
#endif

struct TAuxListDofNames
{
    const char* name;
    bool        dofs[6];
};
const TAuxListDofNames listDofNames[] = {
    {"DX", {1, 0, 0, 0, 0, 0}},       {"DY", {0, 1, 0, 0, 0, 0}},
    {"DZ", {0, 0, 1, 0, 0, 0}},       {"RX", {0, 0, 0, 1, 0, 0}},
    {"RY", {0, 0, 0, 0, 1, 0}},       {"RZ", {0, 0, 0, 0, 0, 1}},
    {"ALL", {1, 1, 1, 1, 1, 1}},      {"DXDYDZRXRYRZ", {1, 1, 1, 1, 1, 1}},
    {"DXDYDZ", {1, 1, 1, 0, 0, 0}},   {"RXRYRZ", {0, 0, 0, 1, 1, 1}},
    {"DXDY", {1, 1, 0, 0, 0, 0}},     {"DXDZ", {1, 0, 1, 0, 0, 0}},
    {"DYDZ", {0, 1, 1, 0, 0, 0}},     {"DXDYRZ", {1, 1, 0, 0, 0, 1}},
    {"DXRZ", {1, 0, 0, 0, 0, 1}},     {"DYRZ", {0, 1, 0, 0, 0, 1}},
    {"DXDYRXRZ", {1, 1, 0, 1, 0, 1}},
};
const size_t listDofNamesCount = sizeof(listDofNames) / sizeof(listDofNames[0]);

// Fwd decl:
#if 0
void parseParams(
    const vector_string_t& tokens, TParsedParams& params, size_t first_idx = 1);
bool replace_paramsets(
    mrpt::containers::yaml&                              inout_params,
    const std::map<std::string, mrpt::containers::yaml>& user_param_sets,
    const EvaluationContext&                             eval_context);
#endif

#define REPORT_ERROR(_MSG)                                           \
    if (ctx.err_msgs)                                                \
    {                                                                \
        ctx.err_msgs->push_back(openbeam::format(                    \
            "Line %u: %s", ctx.lin_num, std::string(_MSG).c_str())); \
    }                                                                \
    else                                                             \
    {                                                                \
        std::cerr << openbeam::format(                               \
            "Line %u: %s", ctx.lin_num, std::string(_MSG).c_str());  \
        return false;                                                \
    }

#define REPORT_WARNING(_MSG)                                                   \
    if (ctx.warn_msgs)                                                         \
    {                                                                          \
        ctx.warn_msgs->push_back(openbeam::format(                             \
            "Line %u: (Warning) %s", ctx.lin_num, std::string(_MSG).c_str())); \
    }                                                                          \
    else                                                                       \
    {                                                                          \
        std::cerr << openbeam::format(                                         \
            "Line %u: (Warning) %s", ctx.lin_num, std::string(_MSG).c_str());  \
    }

#define EVALUATE_EXPRESSION(_IN_EXPR, _OUT_VAL) \
    eval_context.evaluate(_IN_EXPR, _OUT_VAL)

#define REPLACE_PARAMSETS(_PARAMS) \
    replace_paramsets(_PARAMS, user_param_sets, eval_context)

// -------------------------------------------------
//              loadFromStream
// -------------------------------------------------
bool CFiniteElementProblem::loadFromStream(
    std::istream& is, const mrpt::optional_ref<vector_string_t>& errMsg,
    const mrpt::optional_ref<vector_string_t>& warnMsg)
{
    try
    {
        auto f = mrpt::containers::yaml::FromStream(is);
        return internal_loadFromYaml(f, errMsg, warnMsg);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        if (errMsg) errMsg.value().get().push_back(e.what());
        return false;
    }
}

// -------------------------------------------------
//              loadFromFile
// -------------------------------------------------
bool CFiniteElementProblem::loadFromFile(
    const std::string& file, const mrpt::optional_ref<vector_string_t>& errMsg,
    const mrpt::optional_ref<vector_string_t>& warnMsg)
{
    try
    {
        mrpt::containers::yaml f;

        if (file == "-")
        {
            // File "-" means: console input
            f.loadFromStream(std::cin);
        }
        else
        {
            f.loadFromFile(file);
        }

        return internal_loadFromYaml(f, errMsg, warnMsg);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        if (errMsg) errMsg.value().get().push_back(e.what());
        return false;
    }
}

// -------------------------------------------------
//              internal_loadFromYaml
// -------------------------------------------------
bool CFiniteElementProblem::internal_loadFromYaml(
    const mrpt::containers::yaml&              f,
    const mrpt::optional_ref<vector_string_t>& err_msgs,
    const mrpt::optional_ref<vector_string_t>& warn_msgs)
{
    mrpt::system::CTimeLoggerEntry tle(openbeam::timelog, "parseFile");

    // Clear msgs, if any:
    if (err_msgs) err_msgs->get().clear();
    if (warn_msgs) warn_msgs->get().clear();

    // Clear previous contents:
    this->clear();

    // Open OK: go and parse the file:
    // -------------------------------------------
    enum TSection
    {
        sNone = 0,
        sGeometry,
        sLoads
    };

    // List of existing user variables declared in the script:
    // ------------------------------------------------------------
    std::map<std::string, mrpt::containers::yaml> user_param_sets;

    EvaluationContext ctx;
    ctx.warn_msgs = warn_msgs ? &warn_msgs->get() : nullptr;
    ctx.err_msgs  = err_msgs ? &err_msgs->get() : nullptr;

    try
    {
        ASSERTMSG_(
            f.isMap(), "YAML file root element must be a map/dictionary");

        // ---------------------------
        // Parameters
        // ---------------------------
        internal_parser1_Parameters(f, ctx);

        // ---------------------------
        // Beam sections
        // ---------------------------
        internal_parser2_BeamSections(f, ctx);

        // ----------------------------------------
        // Geometry: nodes, elements, constraints
        // ----------------------------------------
        internal_parser3_nodes(f, ctx);
        internal_parser4_elements(f, ctx);

        // Before adding constrains, we must analyze the list of DoFs in the
        // problem:
        OB_MESSAGE(4)
            << "Computing list of DoFs before introducing constraints.\n";
        updateElementsOrientation();
        updateListDoFs();

        internal_parser5_constraints(f, ctx);

        // ----------------------------------------
        // Loads: on nodes, on elements
        // ----------------------------------------
        internal_parser6_node_loads(f, ctx);
        internal_parser7_element_loads(f, ctx);

        // Return OK only if no error messages were emmitted:
        if (err_msgs)
            return err_msgs->get().empty();
        else
            // Otherwise, if we're here it's because all was OK:
            return true;
    }
    catch (const std::exception& e)
    {
        const std::string sErr(e.what());
        if (sErr.empty())
        {
            // If we catch an exception is because err_msgs=nullptr but we
            // found an error, which was already dumped to cerr. So just
            // return false and we're done.
            return false;
        }
        else
        {
            if (err_msgs)
                err_msgs->get().push_back(sErr);
            else
                std::cerr << sErr << std::endl;

            return false;
        }
    }
}

num_t EvaluationContext::evaluate(const std::string& sVarVal) const
{
    OB_MESSAGE(5) << "[evaluate] Line: " << lin_num
                  << " Expression: " << sVarVal << "..." << std::endl;

    num_t val = openbeam::evaluate(sVarVal, parameters, lin_num);

    OB_MESSAGE(5) << " ==> " << val << std::endl;
    return val;
}

#if 0
/** Parse and classify "k1=v1,k2=v2,a,b,c"
 */
void parseParams(
    const vector_string_t& tokens, TParsedParams& params, size_t idx)
{
    const size_t N = tokens.size();

    params.key_vals.clear();
    params.rest_values.clear();

    for (; idx < N; ++idx)
    {
        vector_string_t parts;
        mrpt::system::tokenize(tokens[idx], "=", parts);
        if (parts.size() == 2)
            params.key_vals[parts[0]] = parts[1];
        else
            params.rest_values.push_back(tokens[idx]);
    }
}

/** Search for an entry "paramset=<id>" in \a inout_params and in that case
 * replace it by the corresponding entry
 * Return false only if there was an error if there's not "err_msgs".
 * Should only process the case of returning true and do nothing else on return
 * false to handle the error.
 */
bool replace_paramsets(
    mrpt::containers::yaml&                              inout_params,
    const std::map<std::string, mrpt::containers::yaml>& user_param_sets,
    const EvaluationContext&                             eval_context)
{
    mrpt::containers::yaml::iterator it = inout_params.find("paramset");
    if (it == inout_params.end()) return true;  // No paramset, go on.

    auto itPar = user_param_sets.find(it->second);
    if (itPar == user_param_sets.end())
    {
        REPORT_ERROR("Usage of undefined PARAMSET id.");
        if (!eval_context.err_msgs)
            throw std::runtime_error("");
        else
            return false;
    }
    else
    {
        // OK, replace:
        inout_params.erase(it);
        inout_params.insert(itPar->second.begin(), itPar->second.end());
        return true;
    }
}
#endif

void CFiniteElementProblem::internal_parser1_Parameters(
    const mrpt::containers::yaml& f, EvaluationContext& ctx) const
{
    try
    {
        if (!f.has("parameters")) return;  // none.

        auto p = f["parameters"];
        ASSERT_(p.isMap());

        for (const auto& kv : p.asMap())
        {
            const auto name   = kv.first.as<std::string>();
            const auto k      = kv.first.as<std::string>();
            const auto valStr = kv.second.as<std::string>();

            if (ctx.parameters.count(k) != 0)
                throw std::runtime_error(mrpt::format(
                    "[Line: %i] parameter with name '%s' was already "
                    "defined.",
                    kv.first.marks.line + 1, k.c_str()));

            ctx.lin_num       = kv.second.marks.line;
            ctx.parameters[k] = ctx.evaluate(valStr);
            OB_MESSAGE(4) << "[parser1] Defined new parameter: " << k << "="
                          << ctx.parameters[k] << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'parameters' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser2_BeamSections(
    const mrpt::containers::yaml& f, EvaluationContext& ctx) const
{
    try
    {
        if (!f.has("beam_sections"))
            throw std::runtime_error(
                "Cannot find mandatory 'beam_sections' section in YAML "
                "file");

        auto p = f["beam_sections"];
        ASSERT_(p.isSequence());

        for (const auto& e : p.asSequence())
        {
            if (!e.isMap())
                throw std::runtime_error(
                    "Each entry in the 'beam_sections' section must be a "
                    "map/dictionary");

            const auto& ne = e.asMap();

            const auto sectionName = ne.at("name").as<std::string>();

            auto& bsp = ctx.beamSectionParameters[sectionName];
            bsp       = mrpt::containers::yaml::Map();

            for (const auto& kv : ne)
            {
                const auto k = kv.first.as<std::string>();
                if (k == "name") continue;

                if (bsp.has(k))
                    throw std::runtime_error(mrpt::format(
                        "[Line: %i] beam parameter with name '%s' was "
                        "already "
                        "defined.",
                        kv.first.marks.line + 1, k.c_str()));

                const double val = ctx.evaluate(kv.second.as<std::string>());
                bsp[k]           = val;
            }

            OB_MESSAGE(4) << "[parser2] Defined new beamSection named '"
                          << sectionName << "' with " << bsp.size()
                          << " properties." << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'beam_sections' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser3_nodes(
    const mrpt::containers::yaml& f, EvaluationContext& ctx)
{
    try
    {
        if (!f.has("nodes"))
            throw std::runtime_error(
                "Cannot find mandatory 'nodes' section in YAML file");

        auto p = f["nodes"];
        ASSERT_(p.isSequence());
        ASSERT_(!p.asSequence().empty());

        for (const auto& e : p.asSequence())
        {
            ASSERT_(e.isMap());
            const auto& em = e.asMap();

            // - {id: 0, coords: [0   ,  0], rot_z=30, label: A}

            const double dID = ctx.evaluate(em.at("id").as<std::string>());
            const auto   id  = mrpt::round(dID);

            // auto-grow list of nodes:
            if (id >= getNumberOfNodes()) setNumberOfNodes(id + 1);

            const auto& seqCoords = em.at("coords").asSequence();
            ASSERTMSG_(
                seqCoords.size() == 2 || seqCoords.size() == 3,
                mrpt::format(
                    "Near line %i: `coords` of node must have 2 [x,y] or 3 "
                    "[x,y,z] numbers",
                    e.marks.line + 1));

            num_t x = 0, y = 0, z = 0, rot_x = 0, rot_y = 0, rot_z = 0;
            x = ctx.evaluate(seqCoords.at(0).as<std::string>());
            y = ctx.evaluate(seqCoords.at(1).as<std::string>());
            if (seqCoords.size() >= 3)
                z = ctx.evaluate(seqCoords.at(2).as<std::string>());

            if (em.count("rot_x") != 0)
                rot_x = DEG2RAD(ctx.evaluate(em.at("rot_x").as<std::string>()));
            if (em.count("rot_y") != 0)
                rot_y = DEG2RAD(ctx.evaluate(em.at("rot_y").as<std::string>()));
            if (em.count("rot_z") != 0)
                rot_z = DEG2RAD(ctx.evaluate(em.at("rot_z").as<std::string>()));

            std::string nodeLabel;
            if (em.count("label") != 0)
                nodeLabel = em.at("label").as<std::string>();

            // We have all the data, do insert the new node:

            // Set node pose:
            TRotationTrans3D node_pose(x, y, z, rot_x, rot_y, rot_z);

            setNodePose(id, node_pose);
            m_node_labels[id] = nodeLabel;

            OB_MESSAGE(3) << "Adding node #" << id << " at (" << x << "," << y
                          << "," << z << "," << rot_x << "," << rot_y << ","
                          << rot_z << ") label='" << nodeLabel << "'\n";
        }

        // check no undefined node IDs:
        std::string unusedNodesErrorMsg;
        for (size_t i = 0; i < m_node_defined.size(); i++)
        {
            if (!m_node_defined[i].used)
            {
                unusedNodesErrorMsg += std::to_string(i);
                unusedNodesErrorMsg += " ";
            }
        }
        if (!unusedNodesErrorMsg.empty())
        {
            throw std::runtime_error(mrpt::format(
                "Undefined node IDs: %s", unusedNodesErrorMsg.c_str()));
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error("Errors found in 'nodes' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser4_elements(
    const mrpt::containers::yaml& f, EvaluationContext& ctx)
{
    try
    {
        if (!f.has("elements"))
            throw std::runtime_error(
                "Cannot find mandatory 'elements' section in YAML file");

        auto p = f["elements"];
        ASSERT_(p.isSequence());
        ASSERT_(!p.asSequence().empty());

        for (const auto& e : p.asSequence())
        {
            ASSERT_(e.isMap());
            const auto& em = e.asMap();

            // - {type: BEAM2D_AA, nodes: [0, 1], section: MY_BAR}

            // Element type:
            const std::string eType = em.at("type").as<std::string>();

            auto el = CElement::createElementByName(eType);
            if (!el)
            {
                throw std::runtime_error(mrpt::format(
                    "Line %i: Unknown element type '%s'", e.marks.line + 1,
                    eType.c_str()));
            }

            // connected nodes:
            ASSERT_(em.at("nodes").isSequence());
            ASSERT_GE_(em.at("nodes").asSequence().size(), 2U);

            const auto seqNodes = em.at("nodes").asSequence();
            if (seqNodes.size() != el->conected_nodes_ids.size())
            {
                throw std::runtime_error(mrpt::format(
                    "Line %i: Element of type '%s' expects %u connected "
                    "nodes, "
                    "but %u provided",
                    e.marks.line + 1, eType.c_str(),
                    static_cast<unsigned int>(seqNodes.size()),
                    static_cast<unsigned int>(el->conected_nodes_ids.size())));
            }
            for (size_t i = 0; i < el->conected_nodes_ids.size(); i++)
                el->conected_nodes_ids.at(i) =
                    ctx.evaluate(seqNodes.at(i).as<std::string>());

            // And process the rest of params:
            if (em.count("section") != 0)
            {
                const std::string sectionName =
                    em.at("section").as<std::string>();
                if (ctx.beamSectionParameters.count(sectionName) == 0)
                {
                    throw std::runtime_error(mrpt::format(
                        "Line %i: Element of type '%s' uses section '%s' which "
                        "was "
                        "not defined.",
                        e.marks.line + 1, eType.c_str(), sectionName.c_str()));
                }

                auto& bsp = ctx.beamSectionParameters.at(sectionName);
                el->loadParamsFromSet(bsp, ctx);
            }
            else
            {
                // Load directly from YAML params:
                mrpt::containers::yaml ps = mrpt::containers::yaml::Map();
                for (const auto& kv : em)
                    ps[kv.first.as<std::string>()] = kv.second;

                el->loadParamsFromSet(ps, ctx);
            }

            insertElement(el);

            OB_MESSAGE(3) << "Adding element of type " << eType
                          << " connected to nodes "
                          << mrpt::containers::sprintf_vector(
                                 "%u", el->conected_nodes_ids)
                          << ": " << el->asString() << "\n";
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'elements' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser5_constraints(
    const mrpt::containers::yaml& f, EvaluationContext& ctx)
{
    try
    {
        if (!f.has("constraints"))
            throw std::runtime_error(
                "Cannot find mandatory 'constraints' section in YAML file");

        auto p = f["constraints"];
        ASSERT_(p.isSequence());
        ASSERT_(!p.asSequence().empty());

        for (const auto& e : p.asSequence())
        {
            ASSERT_(e.isMap());
            const auto& em = e.asMap();

            const std::string reqFields[2] = {"node", "dof"};
            for (const auto& f : reqFields)
            {
                if (!em.count(f))
                {
                    throw std::runtime_error(mrpt::format(
                        "Line %i: Missing required '%s' entry",
                        e.marks.line + 1, f.c_str()));
                }
            }

            // - {node: 0, dof: DXDY}
            // - {node: 0, dof: DXDY, value=0.01}
            const unsigned int nodeId =
                mrpt::round(ctx.evaluate(em.at("node").as<std::string>()));
            ASSERT_(nodeId < getNumberOfNodes());

            const std::string sDof      = em.at("dof").as<std::string>();
            num_t             constrVal = 0;
            if (em.count("value") != 0)
                constrVal = ctx.evaluate(em.at("value").as<std::string>());

            bool found = false;
            for (size_t i = 0; i < listDofNamesCount; i++)
            {
                if (strCmpI(listDofNames[i].name, sDof))
                {
                    for (int k = 0; k < 6; k++)
                        if (listDofNames[i].dofs[k])
                        {
                            const size_t globalIdxDOF =
                                this->getDOFIndex(nodeId, DoF_index(k));
                            if (globalIdxDOF != string::npos)
                            {
                                this->insertConstraint(globalIdxDOF, constrVal);

                                OB_MESSAGE(4)
                                    << "Adding constraint in DoF=" << sDof
                                    << " of node " << nodeId
                                    << " value=" << constrVal << "\n";
                            }
                            else
                            {
                                if (ctx.warn_unused_constraints)
                                {
                                    ctx.lin_num = e.marks.line + 1;

                                    REPORT_WARNING(mrpt::format(
                                        "Constraint ignored since the DoF %i "
                                        "is not considered in the problem "
                                        "geometry.",
                                        k));
                                }
                            }
                        }
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error(mrpt::format(
                    "Line %i: Field 'dof' has an invalid value='%s'",
                    e.marks.line + 1, sDof.c_str()));
            }
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'constraints' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser6_node_loads(
    const mrpt::containers::yaml& f, EvaluationContext& ctx)
{
    try
    {
        if (!f.has("node_loads")) return;

        auto p = f["node_loads"];
        ASSERT_(p.isSequence());

        for (const auto& e : p.asSequence())
        {
            ASSERT_(e.isMap());
            const auto& em = e.asMap();

            const std::string reqFields[3] = {"node", "dof", "value"};
            for (const auto& f : reqFields)
            {
                if (!em.count(f))
                {
                    throw std::runtime_error(mrpt::format(
                        "Line %i: Missing required '%s' entry",
                        e.marks.line + 1, f.c_str()));
                }
            }

            // - {node: 2, dof: DX, value: +P}
            const unsigned int nodeId =
                mrpt::round(ctx.evaluate(em.at("node").as<std::string>()));
            ASSERT_(nodeId < getNumberOfNodes());

            const std::string sDof = em.at("dof").as<std::string>();
            num_t loadVal = ctx.evaluate(em.at("value").as<std::string>());

            bool found = false;
            for (size_t i = 0; i < listDofNamesCount; i++)
            {
                if (strCmpI(listDofNames[i].name, sDof))
                {
                    for (int k = 0; k < 6; k++)
                        if (listDofNames[i].dofs[k])
                        {
                            const size_t globalIdxDOF =
                                this->getDOFIndex(nodeId, DoF_index(k));
                            if (globalIdxDOF != string::npos)
                            {
                                this->addLoadAtDOF(globalIdxDOF, loadVal);

                                OB_MESSAGE(4) << "Adding node load on node "
                                              << nodeId << " dof=" << sDof
                                              << " value=" << loadVal << "\n";
                            }
                            else
                            {
                            }
                        }
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                throw std::runtime_error(mrpt::format(
                    "Line %i: Node load applied in dof='%s' which has not been "
                    "included in the problem (is it a free degree of freedom?)",
                    e.marks.line + 1, sDof.c_str()));
            }
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'node_loads' section, aborting.");
    }
}

void CFiniteElementProblem::internal_parser7_element_loads(
    const mrpt::containers::yaml& f, EvaluationContext& ctx)
{
    try
    {
        if (!f.has("element_loads")) return;

        auto p = f["element_loads"];
        ASSERT_(p.isSequence());

        CStructureProblem* myObj = dynamic_cast<CStructureProblem*>(this);
        if (!myObj)
        {
            throw std::runtime_error(
                "`element_loads` only applicable to "
                "openbeam::CStructureProblem objects");
        }

        for (const auto& e : p.asSequence())
        {
            ASSERT_(e.isMap());
            const auto& em = e.asMap();

            const std::string reqFields[2] = {"element", "type"};
            for (const auto& f : reqFields)
            {
                if (!em.count(f))
                {
                    throw std::runtime_error(mrpt::format(
                        "Line %i: Missing required '%s' entry",
                        e.marks.line + 1, f.c_str()));
                }
            }

            // #- {element: 0, type: TEMPERATURE, deltaT: 20}
            const unsigned int elementId =
                mrpt::round(ctx.evaluate(em.at("element").as<std::string>()));
            ASSERT_(elementId < getNumberOfElements());

            const std::string sType = em.at("type").as<std::string>();

            auto load = CLoadOnBeam::createLoadByName(sType);
            if (!load)
            {
                throw std::runtime_error(mrpt::format(
                    "Line %i: Unknown element load type='%s'", e.marks.line + 1,
                    sType.c_str()));
            }
            else
            {
                // Process the rest of
                // params:
                load->loadParamsFromSet(e, ctx);

                // And add:
                myObj->addLoadAtBeam(elementId, load);
            }
        }
    }
    catch (const std::exception& e)
    {
        if (ctx.err_msgs)
            ctx.err_msgs->push_back(e.what());
        else
            std::cerr << e.what();
        throw std::runtime_error(
            "Errors found in 'element_loads' section, aborting.");
    }
}
