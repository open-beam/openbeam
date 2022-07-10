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

#include <float.h>
#include <openbeam/types.h>

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>

using namespace std;
using namespace openbeam;

int VERB_LEVEL = 1;

/** Changes the overall library verbosity level: 0=quiet, 1=info
 * 2=verbose. \sa getVerbosityLevel
 */
void openbeam::setVerbosityLevel(int verbose_level)
{
    VERB_LEVEL = verbose_level;
}

/** Get the current verbosity level of the library.
 * \sa setVerbosityLevel
 */
int openbeam::getVerbosityLevel() { return VERB_LEVEL; }

/** Return true if the two strings are equal (case insensitive)  */
bool openbeam::strCmpI(const std::string& s1, const std::string& s2)
{
#ifdef _WIN32
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
    return !::_strcmpi(s1.c_str(), s2.c_str());
#else
    return !::strcmpi(s1.c_str(), s2.c_str());
#endif
#else
    return !::strcasecmp(s1.c_str(), s2.c_str());
#endif
}

/** Returns the numeric representation of a string, or raises an exception if
 * it's not a valid number */
num_t openbeam::str2num(const std::string& s)
{
    std::stringstream ss(s);
    num_t             val;
    if ((ss >> val).fail())
        throw std::runtime_error(
            std::string("'") + s + std::string("' is not a valid number."));
    return val;
}
