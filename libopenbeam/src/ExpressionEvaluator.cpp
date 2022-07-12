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

double openbeam::evaluate(
    const std::string& expr, const std::map<std::string, double>& userSymbols,
    int lineNumber)
{
    try
    {
        mrpt::expr::CRuntimeCompiledExpression rce;
        rce.compile(expr, userSymbols, expr);
        return rce.eval();
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(
            mrpt::format("[Line: %i] %s", lineNumber + 1, e.what()));
    }
}
