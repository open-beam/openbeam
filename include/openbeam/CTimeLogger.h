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

#include <stack>
#include "CTicTac.h"

namespace openbeam
{
using namespace std;

/** A versatile "profiler" that logs the time spent within each pair of calls to
 * enter(X)-leave(X), among other stats. The results can be dumped to cout or to
 * Visual Studio's output panel. Recursive methods are supported with no
 * problems, that is, calling "enter(X) enter(X) ... leave(X) leave(X)".
 *
 *  This class can be also used to monitorize min/mean/max/total stats of any
 * user-provided parameters via the method CTimeLogger::registerUserMeasure()
 *
 * \sa CTimeLoggerEntry
 *
 * \note The default behavior is dumping all the information at destruction.
 */
class CTimeLogger
{
   private:
    CTicTac m_tictac;
    bool    m_enabled;

    //! Data of all the calls:
    struct TCallData
    {
        TCallData();

        size_t                        n_calls;
        double                        min_t, max_t, mean_t;
        stack<double, vector<double>> open_calls;
        bool                          has_time_units;
    };

    map<string, TCallData> m_data;

    void   do_enter(const char* func_name);
    double do_leave(const char* func_name);

   public:
    /** Data of each call section: # of calls, minimum, maximum, average and
     * overall execution time (in seconds) \sa getStats */
    struct TCallStats
    {
        size_t n_calls;
        double min_t, max_t, mean_t, total_t;
    };

    CTimeLogger(bool enabled = true);  //! Default constructor
    virtual ~CTimeLogger();  //!< Destructor
    std::string getStatsAsText(const size_t column_width = 80)
        const;  //!< Dump all stats to a multi-line text string. \sa
                //!< dumpAllStats, saveToCVSFile
    void getStats(std::map<std::string, TCallStats>& out_stats)
        const;  //!< Returns all the current stats as a map: section_name =>
                //!< stats. \sa getStatsAsText, dumpAllStats, saveToCVSFile
    void dumpAllStats(const size_t column_width = 80)
        const;  //!< Dump all stats through the CDebugOutputCapable interface.
                //!< \sa getStatsAsText, saveToCVSFile
    void clear(
        bool deep_clear =
            false);  //!< Resets all stats. By default (deep_clear=false), all
                     //!< section names are remembered (not freed) so the cost
                     //!< of creating upon the first next call is avoided.
    void enable(bool enabled = true) { m_enabled = enabled; }
    void disable() { m_enabled = false; }
    void saveToCSVFile(const std::string& csv_file)
        const;  //!< Dump all stats to a Comma Separated Values (CSV) file. \sa
                //!< dumpAllStats
    void registerUserMeasure(const char* event_name, const double value);

    /** Start of a named section \sa enter */
    inline void enter(const char* func_name)
    {
        if (m_enabled) do_enter(func_name);
    }
    /** End of a named section \return The ellapsed time, in seconds or 0 if
     * disabled. \sa enter */
    inline double leave(const char* func_name)
    {
        return m_enabled ? do_leave(func_name) : 0;
    }
    /** Return the mean execution time of the given "section", or 0 if it hasn't
     * ever been called "enter" with that section name */
    double getMeanTime(const std::string& name) const;
};  // End of class def.

/** A safe way to call enter() and leave() of a mrpt::utils::CTimeLogger upon
 * construction and destruction of this auxiliary object, making sure that
 * leave() will be called upon exceptions, etc. Usage: \code CTimeLogger logger;
 *    // ...
 *    { // Start of scope to be monitorized
 *       CTimeLoggerEntry tle(logger,"operation-name");
 *
 *       // do whatever
 *
 *    } // End of scope
 * \endcode
 */
struct CTimeLoggerEntry
{
    CTimeLoggerEntry(CTimeLogger& logger, const char* section_name);
    ~CTimeLoggerEntry();
    CTimeLogger& m_logger;
    const char*  m_section_name;
};

}  // namespace openbeam
