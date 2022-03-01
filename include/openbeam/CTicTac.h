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

#include "types.h"

namespace openbeam
{
/** This class implements a high-performance stopwatch.
 *  Typical resolution is about 1e-6 seconds.
 *  \note The class is named after the Spanish equivalent of "Tic-Toc" ;-)
 */
class CTicTac
{
   public:
    /** Default constructor. */
    CTicTac();

    /** Destructor. */
    virtual ~CTicTac();

    /** Starts the stopwatch
     * \sa Tac
     */
    void Tic();

    /** Stops the stopwatch
     * \return Returns the ellapsed time in seconds.
     * \sa Tic
     */
    double Tac();

   private:
    unsigned char largeInts[64];

    // Cannot be copied:
    CTicTac(const CTicTac& o);
    CTicTac& operator=(const CTicTac& o);

};  // End of class def.

}  // namespace openbeam
