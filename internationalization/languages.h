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

namespace openbeam
{
namespace localization
{
// This list must be kept synch with "lang_names" in localization.cpp
enum TLanguage
{
    LANG_EN = 0,
    LANG_ES,

    // Always leave this as the last one
    NUMBER_OF_LANGUAGES
};

// Add one declaration for each language declared above.
//  Strings are defined in the files "lang_**.cpp".
extern const char* strs_en[];
extern const char* strs_es[];
}  // namespace localization
}  // namespace openbeam
