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

#include "ExpressionEvaluator.h"

#include <mrpt/core/format.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

#include <stdexcept>

double openbeam::evaluate(
    const std::string& expr, const std::map<std::string, double>& userSymbols, int lineNumber)
{
  try
  {
    mrpt::expr::CRuntimeCompiledExpression rce;
    rce.compile(expr, userSymbols, expr);
    return rce.eval();
  }
  catch (const std::exception& e)
  {
    // Keep only the parser diagnostic, e.g. "Undefined symbol: 'Z'", out
    // of the full exception text (which includes a backtrace):
    std::string msg = e.what();
    const std::string tag = "Error: `";
    if (const auto p = msg.find(tag); p != std::string::npos)
    {
      msg = msg.substr(p + tag.size());
      msg = msg.substr(0, msg.find('`'));
      if (const auto dash = msg.find(" - "); dash != std::string::npos)
      {
        msg = msg.substr(dash + 3);  // drop the "ERR123" code
      }
    }
    throw std::runtime_error(
        mrpt::format("Line %i: %s in expression '%s'", lineNumber + 1, msg.c_str(), expr.c_str()));
  }
}
