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

#include <openbeam/types.h>

//#pragma warning (disable:4786)
#include <stack>
#include <string>

namespace ExpressionEvaluator
{
enum
{
    eval_ok = 0,
    eval_unbalanced,
    eval_invalidoperator,
    eval_invalidoperand,
    eval_evalerr
};

int calculate(
    const std::string& expr, float& r,
    const std::map<std::string, num_t, openbeam::ci_less>& user_symbols);
int calculate(
    const std::string& expr, double& r,
    const std::map<std::string, num_t, openbeam::ci_less>& user_symbols);
}  // namespace ExpressionEvaluator
