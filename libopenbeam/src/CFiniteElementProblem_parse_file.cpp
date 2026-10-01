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
#include <openbeam/CFiniteElementProblem.h>
#include <openbeam/CStructureProblem.h>

#include <array>
#include <charconv>
#include <optional>

#include "ExpressionEvaluator.h"

using namespace openbeam;
using mrpt::containers::yaml;

namespace
{
/** Names accepted in "dof:" fields, and the DoFs (DX DY DZ RX RY RZ) each one
 * refers to. */
struct DofName
{
  const char* name;
  std::array<bool, 6> dofs;
};

const DofName DOF_NAMES[] = {
    {          "DX", {1, 0, 0, 0, 0, 0}},
    {          "DY", {0, 1, 0, 0, 0, 0}},
    {          "DZ", {0, 0, 1, 0, 0, 0}},
    {          "RX", {0, 0, 0, 1, 0, 0}},
    {          "RY", {0, 0, 0, 0, 1, 0}},
    {          "RZ", {0, 0, 0, 0, 0, 1}},
    {         "ALL", {1, 1, 1, 1, 1, 1}},
    {"DXDYDZRXRYRZ", {1, 1, 1, 1, 1, 1}},
    {      "DXDYDZ", {1, 1, 1, 0, 0, 0}},
    {      "RXRYRZ", {0, 0, 0, 1, 1, 1}},
    {        "DXDY", {1, 1, 0, 0, 0, 0}},
    {        "DXDZ", {1, 0, 1, 0, 0, 0}},
    {        "DYDZ", {0, 1, 1, 0, 0, 0}},
    {      "DXDYRZ", {1, 1, 0, 0, 0, 1}},
    {        "DXRZ", {1, 0, 0, 0, 0, 1}},
    {        "DYRZ", {0, 1, 0, 0, 0, 1}},
    {    "DXDYRXRZ", {1, 1, 0, 1, 0, 1}},
};

std::optional<std::array<bool, 6>> parseDofName(const std::string& s)
{
  for (const auto& d : DOF_NAMES)
  {
    if (strCmpI(d.name, s))
    {
      return d.dofs;
    }
  }
  return std::nullopt;
}

/// 1-based line number of a YAML node, for error messages.
int lineOf(const yaml::node_t& n) { return n.marks.line + 1; }

/** Throws if any of the given keys is missing in a YAML map. */
void requireKeys(
    const yaml& item, const yaml::node_t& node, std::initializer_list<const char*> keys)
{
  for (const char* k : keys)
  {
    if (!item.has(k))
    {
      throw std::runtime_error(
          mrpt::format("Line %i: Missing required '%s' entry", lineOf(node), k));
    }
  }
}

/** Returns the sequence of YAML map entries under the given top-level key, or
 * an empty sequence if the key is optional and missing. */
yaml::sequence_t getSequenceOfMaps(const yaml& f, const char* key, bool required, bool allowEmpty)
{
  if (!f.has(key))
  {
    if (required)
    {
      throw std::runtime_error(
          mrpt::format("Cannot find mandatory '%s' section in YAML file", key));
    }
    return {};
  }
  const auto& p = f[key];
  if (!p.isSequence())
  {
    throw std::runtime_error(mrpt::format("The '%s' section must be a sequence", key));
  }
  const auto& seq = p.asSequence();
  if (seq.empty() && !allowEmpty)
  {
    throw std::runtime_error(mrpt::format("The '%s' section must not be empty", key));
  }
  for (const auto& e : seq)
  {
    if (!e.isMap())
    {
      throw std::runtime_error(
          mrpt::format("Line %i: each entry in '%s' must be a map/dictionary", lineOf(e), key));
    }
  }
  return seq;
}

/** Stores the error of a parser section and re-throws a summary. */
[[noreturn]] void failSection(EvaluationContext& ctx, const std::exception& e, const char* section)
{
  if (ctx.err_msgs)
  {
    ctx.err_msgs->push_back(e.what());
  }
  else
  {
    std::cerr << e.what() << "\n";
  }
  throw std::runtime_error(mrpt::format("Errors found in '%s' section, aborting.", section));
}

void reportWarning(const EvaluationContext& ctx, const std::string& msg)
{
  const auto s = mrpt::format("Line %u: (Warning) %s", ctx.lin_num + 1, msg.c_str());
  if (ctx.warn_msgs)
  {
    ctx.warn_msgs->push_back(s);
  }
  else
  {
    std::cerr << s << "\n";
  }
}

}  // namespace

// -------------------------------------------------
//              loadFromStream
// -------------------------------------------------
bool CFiniteElementProblem::loadFromStream(
    std::istream& is,
    const mrpt::optional_ref<vector_string_t>& errMsg,
    const mrpt::optional_ref<vector_string_t>& warnMsg)
{
  try
  {
    const auto f = yaml::FromStream(is);
    return internal_loadFromYaml(f, errMsg, warnMsg);
  }
  catch (const std::exception& e)
  {
    if (errMsg)
    {
      errMsg.value().get().push_back(e.what());
    }
    else
    {
      std::cerr << e.what() << "\n";
    }
    return false;
  }
}

// -------------------------------------------------
//              loadFromFile
// -------------------------------------------------
bool CFiniteElementProblem::loadFromFile(
    const std::string& file,
    const mrpt::optional_ref<vector_string_t>& errMsg,
    const mrpt::optional_ref<vector_string_t>& warnMsg)
{
  try
  {
    yaml f;
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
    if (errMsg)
    {
      errMsg.value().get().push_back(e.what());
    }
    else
    {
      std::cerr << e.what() << "\n";
    }
    return false;
  }
}

// -------------------------------------------------
//              internal_loadFromYaml
// -------------------------------------------------
bool CFiniteElementProblem::internal_loadFromYaml(
    const yaml& f,
    const mrpt::optional_ref<vector_string_t>& err_msgs,
    const mrpt::optional_ref<vector_string_t>& warn_msgs)
{
  mrpt::system::CTimeLoggerEntry tle(openbeam::timelog, "parseFile");

  if (err_msgs)
  {
    err_msgs->get().clear();
  }
  if (warn_msgs)
  {
    warn_msgs->get().clear();
  }

  this->clear();

  EvaluationContext ctx;
  ctx.warn_msgs = warn_msgs ? &warn_msgs->get() : nullptr;
  ctx.err_msgs = err_msgs ? &err_msgs->get() : nullptr;

  try
  {
    ASSERTMSG_(f.isMap(), "YAML file root element must be a map/dictionary");

    internal_parser1_Parameters(f, ctx);
    internal_parser2_BeamSections(f, ctx);
    internal_parser3_nodes(f, ctx);
    internal_parser4_elements(f, ctx);

    // Constraints refer to the list of DoFs in the problem, so build it
    // first:
    OB_MESSAGE(4) << "Computing list of DoFs before introducing constraints.\n";
    updateElementsOrientation();
    updateListDoFs();

    internal_parser5_constraints(f, ctx);
    internal_parser6_node_loads(f, ctx);
    internal_parser7_element_loads(f, ctx);

    return err_msgs ? err_msgs->get().empty() : true;
  }
  catch (const std::exception& e)
  {
    const std::string sErr(e.what());
    if (!sErr.empty())
    {
      if (err_msgs)
      {
        err_msgs->get().push_back(sErr);
      }
      else
      {
        std::cerr << sErr << std::endl;
      }
    }
    return false;
  }
}

num_t EvaluationContext::evaluate(const std::string& sVarVal) const
{
  OB_MESSAGE(5) << "[evaluate] Line: " << lin_num << " Expression: " << sVarVal << "..."
                << std::endl;

  // Fast path for plain numbers, by far the most common case:
  num_t val = 0;
  const char* first = sVarVal.data();
  const char* last = first + sVarVal.size();
  const auto [ptr, ec] = std::from_chars(first, last, val);
  if (ec != std::errc() || ptr != last)
  {
    val = openbeam::evaluate(sVarVal, parameters, lin_num);
  }

  OB_MESSAGE(5) << " ==> " << val << std::endl;
  return val;
}

void CFiniteElementProblem::internal_parser1_Parameters(const yaml& f, EvaluationContext& ctx) const
{
  try
  {
    if (!f.has("parameters"))
    {
      return;
    }

    const auto& p = f["parameters"];
    if (!p.isMap())
    {
      throw std::runtime_error("'parameters' must be a map");
    }

    // Parameters are evaluated in order, so each one may use the previous
    // ones:
    for (const auto& kv : p.node().asMap())
    {
      const auto k = kv.first.as<std::string>();
      if (ctx.parameters.count(k) != 0)
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: parameter with name '%s' was already defined.", lineOf(kv.first), k.c_str()));
      }

      ctx.lin_num = kv.second.marks.line;
      ctx.parameters[k] = ctx.evaluate(kv.second.as<std::string>());
      OB_MESSAGE(4) << "[parser1] Defined new parameter: " << k << "=" << ctx.parameters[k]
                    << std::endl;
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "parameters");
  }
}

void CFiniteElementProblem::internal_parser2_BeamSections(
    const yaml& f, EvaluationContext& ctx) const
{
  try
  {
    const auto seq = getSequenceOfMaps(f, "beam_sections", true /*required*/, true /*allow empty*/);

    for (const auto& e : seq)
    {
      const yaml item(e);
      requireKeys(item, e, {"name"});

      const auto sectionName = item["name"].as<std::string>();
      if (ctx.beamSectionParameters.count(sectionName) != 0)
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: beam section '%s' was already defined.", lineOf(e), sectionName.c_str()));
      }

      auto& bsp = ctx.beamSectionParameters[sectionName];
      bsp = yaml::Map();

      for (const auto& kv : e.asMap())
      {
        const auto k = kv.first.as<std::string>();
        if (k == "name")
        {
          continue;
        }
        if (bsp.has(k))
        {
          throw std::runtime_error(mrpt::format(
              "Line %i: beam parameter '%s' was already defined.", lineOf(kv.first), k.c_str()));
        }
        ctx.lin_num = kv.second.marks.line;
        bsp[k] = ctx.evaluate(kv.second.as<std::string>());
      }

      OB_MESSAGE(4) << "[parser2] Defined new beamSection named '" << sectionName << "' with "
                    << bsp.size() << " properties." << std::endl;
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "beam_sections");
  }
}

void CFiniteElementProblem::internal_parser3_nodes(const yaml& f, EvaluationContext& ctx)
{
  try
  {
    const auto seq = getSequenceOfMaps(f, "nodes", true /*required*/, false /*allow empty*/);

    for (const auto& e : seq)
    {
      // - {id: 0, coords: [0, 0], rot_z: 30, label: A}
      const yaml item(e);
      requireKeys(item, e, {"id", "coords"});
      ctx.lin_num = e.marks.line;

      const double dID = ctx.evaluate(item["id"]);
      if (dID < 0)
      {
        throw std::runtime_error(mrpt::format("Line %i: node IDs must not be negative", lineOf(e)));
      }
      const auto id = static_cast<size_t>(mrpt::round(dID));

      // auto-grow list of nodes:
      if (id >= getNumberOfNodes())
      {
        setNumberOfNodes(id + 1);
      }
      else if (m_node_defined[id].used)
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: node ID %u was already defined", lineOf(e), static_cast<unsigned>(id)));
      }

      const auto& coords = item["coords"];
      if (!coords.isSequence() || (coords.size() != 2 && coords.size() != 3))
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: `coords` of node must have 2 [x,y] or 3 "
            "[x,y,z] numbers",
            lineOf(e)));
      }
      const auto& seqCoords = coords.asSequence();

      num_t x = ctx.evaluate(seqCoords.at(0).as<std::string>());
      num_t y = ctx.evaluate(seqCoords.at(1).as<std::string>());
      num_t z = 0;
      if (seqCoords.size() >= 3)
      {
        z = ctx.evaluate(seqCoords.at(2).as<std::string>());
      }

      num_t rot_x = 0;
      num_t rot_y = 0;
      num_t rot_z = 0;
      if (item.has("rot_x"))
      {
        rot_x = DEG2RAD(ctx.evaluate(item["rot_x"]));
      }
      if (item.has("rot_y"))
      {
        rot_y = DEG2RAD(ctx.evaluate(item["rot_y"]));
      }
      if (item.has("rot_z"))
      {
        rot_z = DEG2RAD(ctx.evaluate(item["rot_z"]));
      }

      std::string nodeLabel;
      if (item.has("label"))
      {
        nodeLabel = item["label"].as<std::string>();
      }

      setNodePose(id, TRotationTrans3D(x, y, z, rot_x, rot_y, rot_z));
      m_node_labels[id] = nodeLabel;

      OB_MESSAGE(3) << "Adding node #" << id << " at (" << x << "," << y << "," << z << "," << rot_x
                    << "," << rot_y << "," << rot_z << ") label='" << nodeLabel << "'\n";
    }

    // Check there are no gaps in the node IDs:
    std::string undefinedIds;
    for (size_t i = 0; i < m_node_defined.size(); i++)
    {
      if (!m_node_defined[i].used)
      {
        undefinedIds += std::to_string(i);
        undefinedIds += " ";
      }
    }
    if (!undefinedIds.empty())
    {
      throw std::runtime_error(mrpt::format("Undefined node IDs: %s", undefinedIds.c_str()));
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "nodes");
  }
}

void CFiniteElementProblem::internal_parser4_elements(const yaml& f, EvaluationContext& ctx)
{
  try
  {
    const auto seq = getSequenceOfMaps(f, "elements", true /*required*/, false /*allow empty*/);

    for (const auto& e : seq)
    {
      // - {type: BEAM2D_AA, nodes: [0, 1], section: MY_BAR}
      const yaml item(e);
      requireKeys(item, e, {"type", "nodes"});
      ctx.lin_num = e.marks.line;

      const std::string eType = item["type"].as<std::string>();

      auto el = CElement::createElementByName(eType);
      if (!el)
      {
        throw std::runtime_error(
            mrpt::format("Line %i: Unknown element type '%s'", lineOf(e), eType.c_str()));
      }

      // Connected nodes:
      const auto& nodes = item["nodes"];
      if (!nodes.isSequence() || nodes.size() != el->conected_nodes_ids.size())
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: Element of type '%s' expects a list of %u "
            "connected nodes",
            lineOf(e), eType.c_str(), static_cast<unsigned int>(el->conected_nodes_ids.size())));
      }
      const auto& seqNodes = nodes.asSequence();
      for (size_t i = 0; i < el->conected_nodes_ids.size(); i++)
      {
        const double n = ctx.evaluate(seqNodes.at(i).as<std::string>());
        if (n < 0 || n >= getNumberOfNodes())
        {
          throw std::runtime_error(
              mrpt::format("Line %i: Element connected to undefined node ID %g", lineOf(e), n));
        }
        el->conected_nodes_ids.at(i) = static_cast<size_t>(mrpt::round(n));
      }

      // Element properties, either from a beam section or inline:
      if (item.has("section"))
      {
        const auto sectionName = item["section"].as<std::string>();
        if (ctx.beamSectionParameters.count(sectionName) == 0)
        {
          throw std::runtime_error(mrpt::format(
              "Line %i: Element of type '%s' uses section '%s' "
              "which was not defined.",
              lineOf(e), eType.c_str(), sectionName.c_str()));
        }
        el->loadParamsFromSet(ctx.beamSectionParameters.at(sectionName), ctx);
      }
      else
      {
        el->loadParamsFromSet(item, ctx);
      }

      insertElement(el);

      OB_MESSAGE(3) << "Adding element of type " << eType << " connected to nodes "
                    << mrpt::containers::sprintf_vector("%u", el->conected_nodes_ids) << ": "
                    << el->asString() << "\n";
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "elements");
  }
}

void CFiniteElementProblem::internal_parser5_constraints(const yaml& f, EvaluationContext& ctx)
{
  try
  {
    const auto seq = getSequenceOfMaps(f, "constraints", true /*required*/, false /*allow empty*/);

    for (const auto& e : seq)
    {
      // - {node: 0, dof: DXDY}
      // - {node: 0, dof: DXDY, value: 0.01}
      const yaml item(e);
      requireKeys(item, e, {"node", "dof"});
      ctx.lin_num = e.marks.line;

      const double nodeId = ctx.evaluate(item["node"]);
      if (nodeId < 0 || nodeId >= getNumberOfNodes())
      {
        throw std::runtime_error(
            mrpt::format("Line %i: constraint on undefined node ID %g", lineOf(e), nodeId));
      }

      const std::string sDof = item["dof"].as<std::string>();
      num_t constrVal = 0;
      if (item.has("value"))
      {
        constrVal = ctx.evaluate(item["value"]);
      }

      const auto dofs = parseDofName(sDof);
      if (!dofs)
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: Field 'dof' has an invalid value='%s'", lineOf(e), sDof.c_str()));
      }

      for (int k = 0; k < 6; k++)
      {
        if (!(*dofs)[k])
        {
          continue;
        }
        if (addNodeConstraint(static_cast<size_t>(mrpt::round(nodeId)), DoF_index(k), constrVal))
        {
          OB_MESSAGE(4) << "Adding constraint in DoF=" << sDof << " of node " << nodeId
                        << " value=" << constrVal << "\n";
        }
        else if (ctx.warn_unused_constraints)
        {
          reportWarning(
              ctx, mrpt::format(
                       "Constraint ignored since the DoF %i is not "
                       "considered in the problem geometry.",
                       k));
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "constraints");
  }
}

void CFiniteElementProblem::internal_parser6_node_loads(const yaml& f, EvaluationContext& ctx)
{
  try
  {
    const auto seq = getSequenceOfMaps(f, "node_loads", false /*required*/, true /*allow empty*/);

    for (const auto& e : seq)
    {
      // - {node: 2, dof: DX, value: +P}
      const yaml item(e);
      requireKeys(item, e, {"node", "dof", "value"});
      ctx.lin_num = e.marks.line;

      const double nodeId = ctx.evaluate(item["node"]);
      if (nodeId < 0 || nodeId >= getNumberOfNodes())
      {
        throw std::runtime_error(
            mrpt::format("Line %i: load on undefined node ID %g", lineOf(e), nodeId));
      }

      const std::string sDof = item["dof"].as<std::string>();
      const num_t loadVal = ctx.evaluate(item["value"]);

      const auto dofs = parseDofName(sDof);
      if (!dofs)
      {
        throw std::runtime_error(mrpt::format(
            "Line %i: Field 'dof' has an invalid value='%s'", lineOf(e), sDof.c_str()));
      }

      for (int k = 0; k < 6; k++)
      {
        if (!(*dofs)[k])
        {
          continue;
        }
        const size_t globalIdxDOF =
            this->getDOFIndex(static_cast<size_t>(mrpt::round(nodeId)), DoF_index(k));
        if (globalIdxDOF != std::string::npos)
        {
          this->addLoadAtDOF(globalIdxDOF, loadVal);
          OB_MESSAGE(4) << "Adding node load on node " << nodeId << " dof=" << sDof
                        << " value=" << loadVal << "\n";
        }
        else
        {
          reportWarning(
              ctx, mrpt::format(
                       "Load ignored since the DoF %i is not "
                       "considered in the problem geometry.",
                       k));
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "node_loads");
  }
}

void CFiniteElementProblem::internal_parser7_element_loads(const yaml& f, EvaluationContext& ctx)
{
  try
  {
    const auto seq =
        getSequenceOfMaps(f, "element_loads", false /*required*/, true /*allow empty*/);
    if (seq.empty())
    {
      return;
    }

    auto* myObj = dynamic_cast<CStructureProblem*>(this);
    if (!myObj)
    {
      throw std::runtime_error(
          "`element_loads` only applicable to "
          "openbeam::CStructureProblem objects");
    }

    for (const auto& e : seq)
    {
      // - {element: 0, type: TEMPERATURE, deltaT: 20}
      const yaml item(e);
      requireKeys(item, e, {"element", "type"});
      ctx.lin_num = e.marks.line;

      const double elementId = ctx.evaluate(item["element"]);
      if (elementId < 0 || elementId >= getNumberOfElements())
      {
        throw std::runtime_error(
            mrpt::format("Line %i: load on undefined element index %g", lineOf(e), elementId));
      }

      const std::string sType = item["type"].as<std::string>();
      auto load = CLoadOnBeam::createLoadByName(sType);
      if (!load)
      {
        throw std::runtime_error(
            mrpt::format("Line %i: Unknown element load type='%s'", lineOf(e), sType.c_str()));
      }
      load->loadParamsFromSet(item, ctx);
      myObj->addLoadAtBeam(static_cast<size_t>(mrpt::round(elementId)), load);
    }
  }
  catch (const std::exception& e)
  {
    failSection(ctx, e, "element_loads");
  }
}
