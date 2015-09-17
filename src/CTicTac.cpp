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
   +---------------------------------------------------------------------------+ */

#include <openbeam/CTicTac.h>

using namespace std;
using namespace openbeam;

#ifdef OB_OS_WINDOWS
#	include <windows.h>
// Macros for easy access to memory with the correct types:
#	define	LARGE_INTEGER_NUMS	reinterpret_cast<LARGE_INTEGER*>(largeInts)
#else
#	include <sys/time.h>
#	define	TIMEVAL_NUMS			reinterpret_cast<struct timeval*>(largeInts)
#endif

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CTicTac::CTicTac()
{
	memset( largeInts, 0, sizeof(largeInts) );

#ifdef OB_OS_WINDOWS
	OBASSERT( sizeof( largeInts ) > 3*sizeof(LARGE_INTEGER) );
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceFrequency(&l[0]);
#else
	OBASSERT( sizeof( largeInts ) > 2*sizeof(struct timeval) );
#endif
	Tic();
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CTicTac::~CTicTac()
{
}

/*---------------------------------------------------------------
						Tic
	Starts the stopwatch
 ---------------------------------------------------------------*/
void	CTicTac::Tic()
{
#ifdef OB_OS_WINDOWS
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceCounter(&l[1]);
#else
	struct timeval* ts = TIMEVAL_NUMS;
    gettimeofday( &ts[0], NULL);
#endif
}

/*---------------------------------------------------------------
						Tac
   Stop. Returns ellapsed time in seconds
 ---------------------------------------------------------------*/
double	CTicTac::Tac()
{
#ifdef OB_OS_WINDOWS
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceCounter( &l[2] );
	return (l[2].QuadPart-l[1].QuadPart)/static_cast<double>(l[0].QuadPart);
#else
	struct timeval* ts = TIMEVAL_NUMS;
    gettimeofday( &ts[1], NULL);

    return ( ts[1].tv_sec - ts[0].tv_sec) +
           1e-6*(  ts[1].tv_usec - ts[0].tv_usec );
#endif
}
