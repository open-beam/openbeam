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

#include "CTextFileLinesParser.h"
#include "ExpressionEvaluator.h"

using namespace std;
using namespace openbeam;
using namespace Eigen;

/** Used to parse "k1=v1,k2=v2,a,b,c" with \a parseParams() */
struct TParsedParams
{
    bool hasKey(const std::string& key) const
    {
        return key_vals.end() != key_vals.find(key);
    }

    TParamSet     key_vals;
    vector_string rest_values;
};

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
void parseParams(
    const vector_string& tokens, TParsedParams& params, size_t first_idx = 1);
bool replace_paramsets(
    TParamSet&                                       inout_params,
    const std::map<std::string, TParamSet, ci_less>& user_param_sets,
    const TEvaluationContext&                        eval_context);

#define REPORT_ERROR(_MSG)                                                    \
    if (eval_context.err_msgs)                                                \
    {                                                                         \
        eval_context.err_msgs->push_back(openbeam::format(                    \
            "Line %u: %s", eval_context.lin_num, std::string(_MSG).c_str())); \
        eval_context.err_msgs->push_back(openbeam::format(                    \
            "Line %u:  --> '%s'", eval_context.lin_num,                       \
            eval_context.lin->c_str()));                                      \
    }                                                                         \
    else                                                                      \
    {                                                                         \
        std::cerr << openbeam::format(                                        \
            "Line %u: %s", eval_context.lin_num, std::string(_MSG).c_str());  \
        std::cerr << openbeam::format(                                        \
            "Line %u:  --> '%s'", eval_context.lin_num,                       \
            eval_context.lin->c_str());                                       \
        return false;                                                         \
    }

#define REPORT_WARNING(_MSG)                                \
    if (eval_context.warn_msgs)                             \
    {                                                       \
        eval_context.warn_msgs->push_back(openbeam::format( \
            "Line %u: (Warning) %s", eval_context.lin_num,  \
            std::string(_MSG).c_str()));                    \
        eval_context.warn_msgs->push_back(openbeam::format( \
            "Line %u:  --> '%s'", eval_context.lin_num,     \
            eval_context.lin->c_str()));                    \
    }                                                       \
    else                                                    \
    {                                                       \
        std::cerr << openbeam::format(                      \
            "Line %u: (Warning) %s", eval_context.lin_num,  \
            std::string(_MSG).c_str());                     \
        std::cerr << openbeam::format(                      \
            "Line %u:  --> '%s'", eval_context.lin_num,     \
            eval_context.lin->c_str());                     \
        return false;                                       \
    }

#define EVALUATE_EXPRESSION(_IN_EXPR, _OUT_VAL) \
    eval_context.parser_evaluate_expression(_IN_EXPR, _OUT_VAL)

#define REPLACE_PARAMSETS(_PARAMS) \
    replace_paramsets(_PARAMS, user_param_sets, eval_context)

// -------------------------------------------------
//              loadFromStream
// -------------------------------------------------
bool CFiniteElementProblem::loadFromStream(
    std::istream& is, vector_string* err_msgs, vector_string* warn_msgs)
{
    CTextFileLinesParser f;

    f.enableCommentFilters(
        true,  // %
        true,  // //
        true  // #
    );

    f.open(is);

    return this->internal_loadFromFile(&f, err_msgs, warn_msgs);
}

// -------------------------------------------------
//              loadFromFile
// -------------------------------------------------
bool CFiniteElementProblem::loadFromFile(
    const std::string& file, vector_string* err_msgs, vector_string* warn_msgs)
{
    CTextFileLinesParser f;

    f.enableCommentFilters(
        true,  // %
        true,  // //
        true  // #
    );

    if (file == "-")
    {
        // File "-" means: console input
        f.open(std::cin);
    }
    else
    {
        auto fil = std::ifstream(file);
        if (fil.is_open())
        {
            f.open(fil);

            if (err_msgs)
                err_msgs->push_back(
                    std::string("Error opening file for reading: ") + file);
            return false;
        }
    }

    return this->internal_loadFromFile(&f, err_msgs, warn_msgs);
}

// -------------------------------------------------
//              internal_loadFromFile
// -------------------------------------------------
bool CFiniteElementProblem::internal_loadFromFile(
    void* f_, vector_string* err_msgs, vector_string* warn_msgs)
{
    CTimeLoggerEntry tle(openbeam::timelog, "parseFile");

    OBASSERT(f_ != NULL)

    CTextFileLinesParser& f = *reinterpret_cast<CTextFileLinesParser*>(f_);

    // Clear msgs, if any:
    if (err_msgs) err_msgs->clear();
    if (warn_msgs) warn_msgs->clear();

    // Warning switches:
    const bool warn_unused_constraints = true;

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
    std::map<std::string, TParamSet, ci_less> user_param_sets;

    TEvaluationContext eval_context;
    eval_context.warn_msgs = warn_msgs;
    eval_context.err_msgs  = err_msgs;

    TSection curSec = sNone;
    string   lin;
    bool     needToRecomputeDOFs = true;

    try
    {
        while (f.getNextLine(lin))
        {
            const unsigned int lin_num =
                static_cast<unsigned int>(f.getCurrentLineNumber());

            // Complete evaluation context:
            eval_context.lin_num = lin_num;
            eval_context.lin     = &lin;

            vector_string toks;
            openbeam::tokenize(lin, ",\r\n", toks);
            if (toks.empty()) continue;

            // Check change of section:
            if (strCmpI("[GEOMETRY]", toks[0]))
            {
                curSec = sGeometry;
                continue;  // Done with this line
            }
            else if (strCmpI("[LOADS]", toks[0]))
            {
                curSec = sLoads;
                continue;  // Done with this line
            }

            // process line commands that may appear at any point in files:
            // -------------------------------------------------------------
            if (strCmpI(toks[0], "VAR"))
            {
                // The sintax must be:
                //  VAR , <NAME> = <VALUE>

                if (toks.size() != 2)
                {
                    REPORT_ERROR(
                        "Expected syntax 'VAR,<NAME>=<VALUE>' [',' "
                        "tokenization]");
                }
                else
                {
                    vector_string parts;
                    tokenize(toks[1], "=", parts);

                    // Parts: [nam] = [VAL]
                    if (parts.size() != 2)
                    {
                        REPORT_ERROR(
                            "Expected syntax 'VAR,<NAME>=<VALUE>' ['=' "
                            "tokenization]");
                    }
                    else
                    {
                        const string sVarName = parts[0];
                        const string sVarVal  = parts[1];
                        // Run the evaluator:
                        num_t val;
                        if (EVALUATE_EXPRESSION(sVarVal, val))
                        {
                            // Declare a new variable:
                            eval_context.user_vars[sVarName] = val;
                            continue;  // Done with this line
                        }
                    }
                }

            }  // end "VAR"
            else if (strCmpI(toks[0], "PARAMSET"))
            {
                // Syntax: PARAMSET,id=<ID>, <NAM1>=<VAL1> [,<NAM2>=<VAL2>
                // [,...] ]
                TParsedParams params;
                parseParams(toks, params);

                if (!params.hasKey("id"))
                {
                    REPORT_ERROR(
                        "Missing mandatory field 'id'. Expected: "
                        "'PARAMSET,id=<ID>, <NAM1>=<VAL1> [,<NAM2>=<VAL2> "
                        "[,...] ]'");
                }
                else
                {
                    // Expand possible PARAMSETs (recursive PARAMSETs...)
                    if (REPLACE_PARAMSETS(params.key_vals))
                    {
                        const std::string sId = params.key_vals["id"];
                        params.key_vals.erase("id");

                        // Add new param set:
                        user_param_sets[sId] = params.key_vals;
                        continue;  // Done with this line
                    }
                }

            }  // end "PARAMSET"

            // process the line depending on the current section:
            // ----------------------------------------------------------
            switch (curSec)
            {
                // =========  GEOMETRY ============
                case sGeometry:
                {
                    if (strCmpI(toks[0], "NODE"))
                    {
                        // Syntax: NODE, ID=<id>, <X>, <Y> [, <Z> , [<ROT_Z>,
                        // [<ROT_Y>, [<ROT_X>] ] ] ]
                        if (toks.size() < 4 || toks.size() > 8)
                        {
                            REPORT_ERROR(
                                "Expected syntax 'NODE, ID=<id>, <X>, <Y>  [, "
                                "<Z> , [<ROT_Z>, [<ROT_Y>, [<ROT_X>] ] ] ]' "
                                "[tokenization]");
                        }
                        else
                        {
                            const bool has_z     = (toks.size() >= 5);
                            const bool has_rot_z = (toks.size() >= 6);
                            const bool has_rot_y = (toks.size() >= 7);
                            const bool has_rot_x = (toks.size() >= 8);

                            // ID=<id>:
                            vector_string parts;
                            tokenize(toks[1], "=", parts);
                            if (parts.size() != 2 || !strCmpI(parts[0], "ID"))
                            {
                                REPORT_ERROR(
                                    "Expected syntax 'NODE, ID=<id>, <X>, <Y>  "
                                    "[, <Z> , [<ROT_Z>, [<ROT_Y>, [<ROT_X>] ] "
                                    "] ]' [problem around the ID]");
                            }
                            else
                            {
                                const string sIDVal = parts[1];
                                num_t        id_val;
                                if (EVALUATE_EXPRESSION(sIDVal, id_val))
                                {
                                    if (id_val != floor(id_val) || id_val < 0)
                                    {
                                        REPORT_ERROR(
                                            "Expected syntax 'NODE, ID=<id>, "
                                            "<X>, <Y>  [, <Z> , [<ROT_Z>, "
                                            "[<ROT_Y>, [<ROT_X>] ] ] ]' [id "
                                            "must be a possitive integer]");
                                    }
                                    else
                                    {
                                        const size_t ID =
                                            static_cast<size_t>(id_val);

                                        num_t X = 0, Y = 0, Z = 0;
                                        num_t rot_X = 0, rot_Y = 0, rot_Z = 0;

                                        if (EVALUATE_EXPRESSION(toks[2], X) &&
                                            EVALUATE_EXPRESSION(toks[3], Y) &&
                                            (!has_z ||
                                             EVALUATE_EXPRESSION(toks[4], Z)) &&
                                            (!has_rot_z ||
                                             EVALUATE_EXPRESSION(
                                                 toks[5], rot_Z)) &&
                                            (!has_rot_y ||
                                             EVALUATE_EXPRESSION(
                                                 toks[6], rot_Y)) &&
                                            (!has_rot_x || EVALUATE_EXPRESSION(
                                                               toks[7], rot_X)))
                                        {
                                            // We have all the data, do insert
                                            // the new node:
                                            const size_t nOldNodes =
                                                this->getNumberOfNodes();
                                            OB_TODO("Check unused IDs")
                                            if (nOldNodes >= ID)
                                                this->setNumberOfNodes(ID + 1);

                                            // Set node pose:
                                            TRotationTrans3D node_pose(
                                                X, Y, Z, DEG2RAD(rot_X),
                                                DEG2RAD(rot_Y), DEG2RAD(rot_Z));
                                            this->setNodePose(ID, node_pose);

                                            needToRecomputeDOFs = true;

                                            OB_MESSAGE(3)
                                                << "Adding node #" << ID
                                                << " at (" << X << "," << Y
                                                << "," << Z << "," << rot_X
                                                << "," << rot_Y << "," << rot_Z
                                                << ")\n";
                                        }
                                    }
                                }
                            }
                        }
                    }  // end NODE
                    else if (strCmpI(toks[0], "ELEMENT"))
                    {
                        // Syntax: ELEMENT, type=<TYPE>, from=<FROM>, to=<TO>,
                        // [,id=ID], [...]
                        TParsedParams params;
                        parseParams(toks, params);

                        if (!params.hasKey("type") || !params.hasKey("from") ||
                            !params.hasKey("to"))
                        {
                            REPORT_ERROR(
                                "Missing mandatory fields: ELEMENT, "
                                "type=<TYPE>, from=<FROM>, to=<TO>, [,id=ID], "
                                "[...]");
                        }
                        else
                        {
                            const std::string& eType = params.key_vals["type"];

                            num_t nFrom = 0, nTo = 0;
                            if (EVALUATE_EXPRESSION(
                                    params.key_vals["from"], nFrom) &&
                                EVALUATE_EXPRESSION(params.key_vals["to"], nTo))
                            {
                                const size_t nNodes = this->getNumberOfNodes();
                                if (nFrom < 0 || nTo < 0 || nFrom >= nNodes ||
                                    nTo >= nNodes)
                                {
                                    REPORT_ERROR(
                                        "'from' and 'to' IDs must be valid IDs "
                                        "of existing nodes");
                                }
                                else
                                {
                                    CElement* el =
                                        CElement::createElementByName(eType);
                                    if (!el)
                                    { REPORT_ERROR("Unknown element type"); }
                                    else
                                    {
                                        // Expand possible PARAMSETs:
                                        if (REPLACE_PARAMSETS(params.key_vals))
                                        {
                                            // Set common params to any element:
                                            OBASSERT(
                                                el->conected_nodes_ids.size() ==
                                                2)
                                            el->conected_nodes_ids[0] =
                                                static_cast<size_t>(nFrom);
                                            el->conected_nodes_ids[1] =
                                                static_cast<size_t>(nTo);
                                            // And process the rest of params:
                                            el->loadParamsFromSet(
                                                params.key_vals, eval_context);
                                            this->insertElement(el);

                                            needToRecomputeDOFs = true;
                                        }
                                    }
                                }
                            }
                        }
                    }  // end ELEMENT
                    else if (strCmpI(toks[0], "CONSTRAINT"))
                    {
                        // Syntax: CONSTRAINT, node=<ID>, dof=<DOF> [,
                        // val=<VAL>]
                        TParsedParams params;
                        parseParams(toks, params);

                        if (REPLACE_PARAMSETS(params.key_vals))
                        {
                            if (!params.hasKey("node") || !params.hasKey("dof"))
                            {
                                REPORT_ERROR(
                                    "Missing mandatory fields: CONSTRAINT, "
                                    "node=<ID>, dof=<DOF> [, val=<VAL>]");
                            }
                            else
                            {
                                const std::string& sNode =
                                    params.key_vals["node"];
                                const std::string& sDof =
                                    params.key_vals["dof"];

                                num_t fNodeId;
                                if (EVALUATE_EXPRESSION(sNode, fNodeId))
                                {
                                    if (fNodeId != floor(fNodeId) ||
                                        fNodeId < 0)
                                    {
                                        REPORT_ERROR(
                                            "Node ID must be a non-negative "
                                            "integer");
                                    }
                                    else
                                    {
                                        // Get the value of the constrained
                                        // displacement (by default=0):
                                        num_t constr_val = 0;

                                        if (!params.hasKey("val") ||
                                            EVALUATE_EXPRESSION(
                                                params.key_vals["val"],
                                                constr_val))
                                        {
                                            const size_t nodeId =
                                                static_cast<size_t>(fNodeId);

                                            // Before adding constrains, we must
                                            // analyze the list of DoFs in the
                                            // problem:
                                            if (needToRecomputeDOFs)
                                            {
                                                OB_MESSAGE(4)
                                                    << "Recomputing list of "
                                                       "DoFs before "
                                                       "introducing new "
                                                       "constraints.\n";
                                                updateElementsOrientation();
                                                updateListDoFs();
                                                needToRecomputeDOFs = false;
                                            }

                                            bool found = false;
                                            for (size_t i = 0;
                                                 i < listDofNamesCount; i++)
                                            {
                                                if (strCmpI(
                                                        listDofNames[i].name,
                                                        sDof))
                                                {
                                                    for (int k = 0; k < 6; k++)
                                                        if (listDofNames[i]
                                                                .dofs[k])
                                                        {
                                                            const size_t
                                                                globalIdxDOF =
                                                                    this->getDOFIndex(
                                                                        nodeId,
                                                                        TDoFIndex(
                                                                            k));
                                                            if (globalIdxDOF !=
                                                                string::npos)
                                                            {
                                                                this->insertConstraint(
                                                                    globalIdxDOF,
                                                                    constr_val);
                                                            }
                                                            else
                                                            {
                                                                if (warn_unused_constraints)
                                                                    REPORT_WARNING(
                                                                        format(
                                                                            "Co"
                                                                            "ns"
                                                                            "tr"
                                                                            "ai"
                                                                            "nt"
                                                                            " i"
                                                                            "gn"
                                                                            "or"
                                                                            "ed"
                                                                            " s"
                                                                            "in"
                                                                            "ce"
                                                                            " t"
                                                                            "he"
                                                                            " D"
                                                                            "oF"
                                                                            " %"
                                                                            "i "
                                                                            "is"
                                                                            " n"
                                                                            "ot"
                                                                            " c"
                                                                            "on"
                                                                            "si"
                                                                            "de"
                                                                            "re"
                                                                            "d "
                                                                            "in"
                                                                            " t"
                                                                            "he"
                                                                            " p"
                                                                            "ro"
                                                                            "bl"
                                                                            "em"
                                                                            " g"
                                                                            "eo"
                                                                            "me"
                                                                            "tr"
                                                                            "y"
                                                                            ".",
                                                                            k))
                                                            }
                                                        }
                                                    found = true;
                                                    break;
                                                }
                                            }

                                            if (!found)
                                            {
                                                REPORT_ERROR(
                                                    "Field 'dof=...' has an "
                                                    "invalid value in "
                                                    "CONSTRAINT");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }  // end CONSTRAINT
                    else if (strCmpI(toks[0], "NODELABEL"))
                    {
                        // Syntax: NODELABEL, ID=<id>, <LABEL>
                        if (toks.size() != 3)
                        {
                            REPORT_ERROR(
                                "Expected syntax 'NODELABEL, ID=<id>, <LABEL>' "
                                "[tokenization]");
                        }
                        else
                        {
                            // ID=<id>:
                            vector_string parts;
                            tokenize(toks[1], "=", parts);
                            if (parts.size() != 2 || !strCmpI(parts[0], "ID"))
                            {
                                REPORT_ERROR(
                                    "Expected syntax 'NODELABEL, ID=<id>, "
                                    "<LABEL>' [problem around the ID]");
                            }
                            else
                            {
                                const string sIDVal = parts[1];
                                num_t        id_val;
                                if (EVALUATE_EXPRESSION(sIDVal, id_val))
                                {
                                    if (id_val != floor(id_val) || id_val < 0)
                                    {
                                        REPORT_ERROR(
                                            "Expected syntax 'NODELABEL, "
                                            "ID=<id>, <LABEL>' [id must be a "
                                            "possitive integer]");
                                    }
                                    else
                                    {
                                        const size_t ID =
                                            static_cast<size_t>(id_val);
                                        const string sLabel = trim(toks[2]);
                                        if (ID >= m_node_labels.size())
                                        {
                                            REPORT_ERROR(
                                                "'NODELABEL: ID was not "
                                                "defined first!");
                                        }
                                        else
                                        {
                                            m_node_labels[ID] = sLabel;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                    {  // Unknown command for this section!
                        REPORT_ERROR("Unknown command for section [GEOMETRY]");
                    }
                }
                break;

                // =========  LOADS ============
                case sLoads:
                {
                    if (strCmpI(toks[0], "LOAD"))
                    {
                        // Syntax: LOAD, node=<ID>, dof=<DOF>, <VALUE>
                        TParsedParams params;
                        parseParams(toks, params);

                        if (REPLACE_PARAMSETS(params.key_vals))
                        {
                            if (!params.hasKey("node") ||
                                !params.hasKey("dof") ||
                                params.rest_values.size() != 1)
                            {
                                REPORT_ERROR(
                                    "Wrong syntax. Expected is: LOAD, "
                                    "node=<ID>, dof=<DOF>, <VALUE>");
                            }
                            else
                            {
                                const std::string& sNode =
                                    params.key_vals["node"];
                                const std::string& sDof =
                                    params.key_vals["dof"];

                                num_t fNodeId;
                                if (EVALUATE_EXPRESSION(sNode, fNodeId))
                                {
                                    if (fNodeId != floor(fNodeId) ||
                                        fNodeId < 0)
                                    {
                                        REPORT_ERROR(
                                            "Node ID must be a non-negative "
                                            "integer");
                                    }
                                    else
                                    {
                                        const size_t nodeId =
                                            static_cast<size_t>(fNodeId);

                                        num_t load_val = 0;

                                        if (EVALUATE_EXPRESSION(
                                                params.rest_values[0],
                                                load_val))
                                        {
                                            // Before adding constrains, we must
                                            // analyze the list of DoFs in the
                                            // problem:
                                            if (needToRecomputeDOFs)
                                            {
                                                updateElementsOrientation();
                                                updateListDoFs();
                                                needToRecomputeDOFs = false;
                                            }

                                            bool found = false;
                                            for (size_t i = 0;
                                                 i < listDofNamesCount; i++)
                                            {
                                                if (strCmpI(
                                                        listDofNames[i].name,
                                                        sDof))
                                                {
                                                    for (int k = 0; k < 6; k++)
                                                        if (listDofNames[i]
                                                                .dofs[k])
                                                        {
                                                            const size_t
                                                                globalIdxDOF =
                                                                    this->getDOFIndex(
                                                                        nodeId,
                                                                        TDoFIndex(
                                                                            k));
                                                            if (globalIdxDOF !=
                                                                string::npos)
                                                            {
                                                                this->addLoadAtDOF(
                                                                    globalIdxDOF,
                                                                    load_val);
                                                            }
                                                            else
                                                            {
                                                                REPORT_WARNING(
                                                                    format(
                                                                        "Load "
                                                                        "ignore"
                                                                        "d "
                                                                        "since "
                                                                        "the "
                                                                        "DoF "
                                                                        "%i is "
                                                                        "not "
                                                                        "consid"
                                                                        "ered "
                                                                        "in "
                                                                        "the "
                                                                        "proble"
                                                                        "m "
                                                                        "geomet"
                                                                        "ry.",
                                                                        k));
                                                            }
                                                        }
                                                    found = true;
                                                    break;
                                                }
                                            }

                                            if (!found)
                                            {
                                                REPORT_ERROR(
                                                    "Field 'dof=...' has an "
                                                    "invalid value in "
                                                    "CONSTRAINT");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }  // end "LOAD"
                    else if (strCmpI(toks[0], "ELOAD"))
                    {
                        CStructureProblem* myObj =
                            dynamic_cast<CStructureProblem*>(this);
                        if (!myObj)
                        {
                            REPORT_ERROR(
                                "ELOAD command only applicable to "
                                "openbeam::CStructureProblem objects.")
                        }
                        else
                        {
                            // Syntax: ELOAD, element=<ID>, type=<TYPE>,
                            // <P1>=<K1>, <P2>=<K2>,...
                            TParsedParams params;
                            parseParams(toks, params);

                            if (!params.hasKey("type") ||
                                !params.hasKey("element"))
                            {
                                REPORT_ERROR(
                                    "Missing mandatory fields: ELOAD, "
                                    "element=<ID>, type=<TYPE>, [...]");
                            }
                            else
                            {
                                const std::string sType =
                                    params.key_vals["type"];
                                const std::string sID =
                                    params.key_vals["element"];
                                params.key_vals.erase("type");
                                params.key_vals.erase("element");

                                num_t fID;

                                if (EVALUATE_EXPRESSION(sID, fID))
                                {
                                    if (fID < 0 || fID != floor(fID))
                                    {
                                        REPORT_ERROR(
                                            "Element ID not a valid ID of "
                                            "existing element.");
                                    }
                                    else
                                    {
                                        const size_t eleIdx =
                                            static_cast<size_t>(fID);

                                        CLoadOnBeam* load =
                                            CLoadOnBeam::createLoadByName(
                                                sType);
                                        if (!load)
                                        { REPORT_ERROR("Unknown load type"); }
                                        else
                                        {
                                            // Expand possible PARAMSETs:
                                            if (REPLACE_PARAMSETS(
                                                    params.key_vals))
                                            {
                                                // Process the rest of params:
                                                load->loadParamsFromSet(
                                                    params.key_vals,
                                                    eval_context);
                                                // And add:
                                                myObj->addLoadAtBeam(
                                                    eleIdx, load);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }  // end ELOAD
                    else
                    {  // Unknown command for this section!
                        REPORT_ERROR("Unknown command for section [LOAD]");
                    }
                }

                default:
                    break;
            };

        };  // end while read line from file

        // Return OK only if no error messages were emmitted:
        if (err_msgs)
            return err_msgs->empty();
        else
            // Otherwise, if we're here it's because all was OK:
            return true;
    }
    catch (std::exception& e)
    {
        const std::string sErr(e.what());
        if (sErr.empty())
        {
            // If we catch an exception is because err_msgs=NULL but we found an
            // error, which was already dumped to cerr. So just return false and
            // we're done.
            return false;
        }
        else
        {
            if (err_msgs)
                err_msgs->push_back(sErr);
            else
                std::cerr << sErr << std::endl;

            return false;
        }
    }
}

// Return false only if there was an error if there's not "err_msgs".
//  Should only process the case of returning true and do nothing else on return
//  false to handle the error.
bool TEvaluationContext::parser_evaluate_expression(
    const std::string& sVarVal, num_t& val) const
{
    OB_MESSAGE(5) << "Line: " << lin_num << " Expression: " << sVarVal
                  << " --> ";

    int ret = ExpressionEvaluator::calculate(sVarVal, val, user_vars);

    if (ret == ExpressionEvaluator::eval_ok)
    {
        OB_MESSAGE(5) << val << endl;
        return true;
    }
    else
    {
        OB_MESSAGE(5) << "ERROR!\n";

        const TEvaluationContext& eval_context =
            *this;  // Just a trick to make the macros REPORT_ERROR below work.

        // Handle possible errors:
        switch (ret)
        {
            case ExpressionEvaluator::eval_unbalanced:
                REPORT_ERROR(format(
                    "Unbalanced expression evaluating variable value: '%s'",
                    sVarVal.c_str()));
                break;
            case ExpressionEvaluator::eval_invalidoperator:
                REPORT_ERROR(format(
                    "Invalid operator evaluating variable value: '%s'",
                    sVarVal.c_str()));
                break;
            case ExpressionEvaluator::eval_invalidoperand:
                REPORT_ERROR(format(
                    "Unknown variable or operand evaluating variable value: "
                    "'%s'",
                    sVarVal.c_str()));
                break;
            case ExpressionEvaluator::eval_evalerr:
            default:
                REPORT_ERROR(format(
                    "Evaluation error evaluating variable value: '%s'",
                    sVarVal.c_str()));
                break;
        }
        if (!err_msgs)
            throw std::runtime_error("");
        else
            return false;
    }
}

/** Parse and classify "k1=v1,k2=v2,a,b,c"
 */
void parseParams(const vector_string& tokens, TParsedParams& params, size_t idx)
{
    const size_t N = tokens.size();

    params.key_vals.clear();
    params.rest_values.clear();

    for (; idx < N; ++idx)
    {
        vector_string parts;
        tokenize(tokens[idx], "=", parts);
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
    TParamSet&                                       inout_params,
    const std::map<std::string, TParamSet, ci_less>& user_param_sets,
    const TEvaluationContext&                        eval_context)
{
    TParamSet::iterator it = inout_params.find("paramset");
    if (it == inout_params.end()) return true;  // No paramset, go on.

    std::map<std::string, TParamSet, ci_less>::const_iterator itPar =
        user_param_sets.find(it->second);
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
