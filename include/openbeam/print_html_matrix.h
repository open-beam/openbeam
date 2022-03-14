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

#include <openbeam/config.h>  // num_t

#include <cmath>
#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace openbeam
{
template <class MATRIX, class ARRAY = MATRIX>
void print_html_matrix(
    std::ostream& out, const MATRIX& K,
    const std::vector<std::string>& row_titles,
    const std::vector<std::string>& col_titles,
    const ARRAY* rows_to_use = nullptr, const ARRAY* cols_to_use = nullptr)
{
    const size_t nCols = K.cols();
    const size_t nRows = K.rows();

    // Determine common factor to make notation clearer:
    int                common_factor_pow = 0;
    std::map<int, int> pow10hits;
    for (size_t i = 0; i < nRows; i++)
    {
        if (rows_to_use && !(*rows_to_use)[i]) continue;

        for (size_t j = 0; j < nCols; j++)
        {
            if (cols_to_use && !(*cols_to_use)[j]) continue;

            const num_t Kijabs = std::abs(K(i, j));

            if (Kijabs > 0)
            {
                const int mag =
                    static_cast<int>(log10(Kijabs));  // Floor to "int"
                pow10hits[mag]++;
            }
            // else: NO, zeroes do not count
        }
    }
    if (!pow10hits.empty())
    {
        std::map<int, int>::const_iterator it_max = pow10hits.begin();
        for (auto it = pow10hits.begin(); it != pow10hits.end(); ++it)
        {
            if (it->second > it_max->second) it_max = it;
        }
        common_factor_pow = it_max->first;
    }

    const num_t common_factor =
        pow(static_cast<num_t>(10), static_cast<num_t>(common_factor_pow));
    const num_t common_factor_inv = 1. / common_factor;

    // Title:
    out << "<table border=\"1\" cellpadding=\"9\" cellspacing=\"0\">\n";

    // 1st row:
    out << "<tr>";
    out << "<td bgcolor=\"#E0E0E0\"> &nbsp; </td>";
    for (size_t i = 0; i < nCols; i++)
    {
        if (cols_to_use && !(*cols_to_use)[i]) continue;

        out << col_titles[i];
    }
    out << "</tr>\n";

    // The rest of rows:
    for (size_t i = 0; i < nRows; i++)
    {
        if (rows_to_use && !(*rows_to_use)[i]) continue;

        out << "<tr>";
        out << row_titles[i];

        for (size_t j = 0; j < nCols; j++)
        {
            if (cols_to_use && !(*cols_to_use)[j]) continue;

            const num_t Kij = K(i, j);
            if (!Kij)
                out << format("<td align=\"right\"> 0 </td>");
            else
                out << format(
                    "<td align=\"right\"> %.3f </td>", Kij * common_factor_inv);
        }

        out << "</tr>\n";
    }
    out << "</table>\n";
    out << format("( &times; 10<sup>%i</sup> )<br>\n", common_factor_pow);
}

}  // namespace openbeam
