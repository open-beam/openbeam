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

#include "CTextFileLinesParser.h"

#include <openbeam/types.h>

#include <sstream>

using namespace openbeam;
using namespace std;

CTextFileLinesParser::CTextFileLinesParser(const std::string& fil)
{
    open(fil);
}

CTextFileLinesParser::CTextFileLinesParser(std::istream& in) { open(in); }
void CTextFileLinesParser::open(std::istream& in)
{
    m_curLineNum = 0;
    m_fileName   = "{std::istream}";
    this->close();
    m_in = &in;
}

void CTextFileLinesParser::open(const std::string& fil)
{
    m_curLineNum = 0;
    m_fileName   = fil;
    this->close();
    auto ifs = std::make_shared<std::ifstream>();
    ifs->clear();
    ifs->open(fil.c_str());
    if (!ifs->is_open())
        throw std::runtime_error(openbeam::format(
            "Error opening file '%s' for reading", fil.c_str()));
    m_my_in = std::shared_ptr<std::istream>(ifs);
    m_in    = m_my_in.get();
}

void CTextFileLinesParser::close()
{
    if (!m_in) return;
    m_my_in.reset();
    m_in = nullptr;
}
void CTextFileLinesParser::rewind()
{
    m_curLineNum = 0;
    m_in->clear();
    m_in->seekg(0);
}

bool CTextFileLinesParser::getNextLine(std::string& out_str)
{
    std::istringstream buf;
    if (getNextLine(buf))
    {
        out_str = buf.str();
        return true;
    }
    out_str.clear();
    return false;
}

bool CTextFileLinesParser::getNextLine(std::istringstream& buf)
{
    OBASSERT(m_in != nullptr);
    while (!m_in->fail())
    {
        std::string lin;
        std::getline(*m_in, lin);
        m_curLineNum++;
        lin = openbeam::trim(lin);
        if (lin.empty()) continue;  // Ignore empty lines.
        // Ignore comments lines, starting with "#" or "//".
        if ((m_filter_SH_comments && openbeam::strStarts(lin, "#")) ||
            (m_filter_C_comments && openbeam::strStarts(lin, "//")) ||
            (m_filter_MATLAB_comments && openbeam::strStarts(lin, "%")))
            continue;
        // Parse the line as a string stream:
        buf.str(lin);
        buf.clear();
        return true;
    };
    return false;
}

size_t CTextFileLinesParser::getCurrentLineNumber() const
{
    return m_curLineNum;
}

void CTextFileLinesParser::enableCommentFilters(
    bool filter_MATLAB_comments, bool filter_C_comments,
    bool filter_SH_comments)
{
    m_filter_MATLAB_comments = filter_MATLAB_comments;
    m_filter_C_comments      = filter_C_comments;
    m_filter_SH_comments     = filter_SH_comments;
}
