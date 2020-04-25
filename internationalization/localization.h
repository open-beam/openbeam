/* +---------------------------------------------------------------------------+
   |              OpenBeam - C++ Finite Element Analysis library               |
   |                                                                           |
   |   Copyright (C) 2010-2013  Jose Luis Blanco Claraco                       |
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

#include <string>

// Common declarations:
#include "lang_strings.h"
#include "languages.h"

namespace openbeam
{
namespace localization
{
/** Selects the language from its 2 letter ISO 639-1 code
 *  ("en","es",...). \return false on unsupported language. */
bool selectLanguage(const std::string& langID);

/// \overload
bool selectLanguage(const TLanguage langID);

/** The main translation function, takes a string code and return its
 * translation for the currently selected language */
const char* _t(const TStringID strID);

}  // namespace localization
}  // namespace openbeam
