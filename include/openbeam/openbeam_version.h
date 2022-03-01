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

#define OPENBEAM_MAJOR_VERSION 0
#define OPENBEAM_MINOR_VERSION 9
#define OPENBEAM_PATCH_VERSION 9

#define OPENBEAM_STR_EXP(__A) #__A
#define OPENBEAM_STR(__A) OPENBEAM_STR_EXP(__A)
#define OPENBEAM_VERSION                                       \
    OPENBEAM_STR(OPENBEAM_MAJOR_VERSION)                       \
    "." OPENBEAM_STR(OPENBEAM_MINOR_VERSION) "." OPENBEAM_STR( \
        OPENBEAM_PATCH_VERSION)
