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

#include <openbeam/types.h>

#include <float.h>
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

/** Changes the overall library verbosity level: 0=quiet, 1=interesting
 * messages, 2=verbose. \sa getVerbosityLevel
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

/*---------------------------------------------------------------
                my_vsnprintf
---------------------------------------------------------------*/
int my_vsnprintf(char* buf, size_t bufSize, const char* format, va_list args)
{
#if defined(_MSC_VER)
#if (_MSC_VER >= 1400)
    // Use a secure version in Visual Studio 2005:
    return ::vsnprintf_s(buf, bufSize, _TRUNCATE, format, args);
#else
    return ::vsprintf(buf, format, args);
#endif
#else
    // Use standard version:
    return ::vsnprintf(buf, bufSize, format, args);
#endif
}

// A sprintf-like function for std::string
string openbeam::format(const char* fmt, ...)
{
    if (!fmt) return string("");

    int          result = -1, length = 1024;
    vector<char> buffer;
    while (result == -1)
    {
        buffer.resize(length + 10);

        va_list args;  // This must be done WITHIN the loop
        va_start(args, fmt);
        result = my_vsnprintf(&buffer[0], length, fmt, args);
        va_end(args);

        // Truncated?
        if (result >= length) result = -1;
        length *= 2;
    }
    string s(&buffer[0]);
    return s;
}

/*---------------------------------------------------------------
                        strtok
---------------------------------------------------------------*/
char* openbeam::strtok(char* str, const char* strDelimit, char** context)
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
    // Use a secure version in Visual Studio 2005:
    return ::strtok_s(str, strDelimit, context);
#else
    // Use standard version:
    return ::strtok(str, strDelimit);
#endif
}

/*---------------------------------------------------------------
                        tokenize
---------------------------------------------------------------*/
void openbeam::tokenize(
    const std::string& inString, const std::string& inDelimiters,
    vector_string& outTokens, bool do_trim_all_parts)
{
    char *nextTok, *context;

    outTokens.clear();

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
    char* dupStr = _strdup(inString.c_str());
#else
    char* dupStr = ::strdup(inString.c_str());
#endif

    nextTok = openbeam::strtok(dupStr, inDelimiters.c_str(), &context);
    while (nextTok != NULL)
    {
        if (do_trim_all_parts)
            outTokens.push_back(openbeam::trim(std::string(nextTok)));
        else
            outTokens.push_back(std::string(nextTok));
        nextTok = openbeam::strtok(NULL, inDelimiters.c_str(), &context);
    };

    free(dupStr);
}

/*---------------------------------------------------------------
                        trim
---------------------------------------------------------------*/
std::string openbeam::trim(const std::string& str)
{
    if (str.empty()) { return std::string(); }
    else
    {
        size_t s = str.find_first_not_of(" \t");
        size_t e = str.find_last_not_of(" \t");
        if (s == std::string::npos || e == std::string::npos)
            return std::string();
        else
            return str.substr(s, e - s + 1);
    }
}
