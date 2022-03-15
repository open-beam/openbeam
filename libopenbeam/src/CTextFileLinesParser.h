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

#include <fstream>
#include <iosfwd>
#include <memory>
#include <string>

namespace openbeam
{
/** A class for parsing text files, returning each non-empty and non-comment
 * line, along its line number. Lines are strip out of leading and trailing
 * whitespaces. By default, lines starting with either "#", "//" or "%" are
 * skipped as comment lines, unless this behavior is explicitly disabled with
 * \a enableCommentFilters.
 */
class CTextFileLinesParser
{
   public:
    /** Default constructor; should call \a open() at some moment later. */
    CTextFileLinesParser() = default;
    /** Constructor for opening a file  \exception std::exception On error
     * opening file */
    explicit CTextFileLinesParser(const std::string& filename);

    /** Constructor for reading from a generic std::istream. Note that a
     * reference to the stream is stored in the object, so it's the user
     * responsibility to make sure the stream is not destroyed before than
     * this object.
     */
    explicit CTextFileLinesParser(std::istream& in);

    /** Open a file (an alternative to the constructor with a file name) */
    void open(const std::string& fil);

    /** Opens for reading a generic std::istream. Note that a
     * reference to the stream is stored in the object, so it's the user
     * responsibility to make sure the stream is not destroyed before than
     * this object.
     */
    void open(std::istream& in);

    /** Close the file (no need to call it normally, the file is closed upon
     * destruction) */
    void close();
    /** Reset the read pointer to the beginning of the file */
    void rewind();

    /** Reads from the file and return the next (non-comment) line, as a
     * std::string
     * \return false on EOF.
     */
    bool getNextLine(std::string& out_str);

    /** Reads from the file and stores the next (non-comment) line into the
     * given stream buffer.
     * \return false on EOF.
     */
    bool getNextLine(std::istringstream& buf);

    /** Return the line number of the last line returned with \a getNextLine */
    size_t getCurrentLineNumber() const;

    /** Enable/disable filtering of lines starting with "%", "//" or "#",
     * respectively. */
    void enableCommentFilters(
        bool filter_MATLAB_comments, bool filter_C_comments,
        bool filter_SH_comments);

   private:
    std::string m_fileName;
    /** Points to either a user-owned object, or to m_my_in */
    std::istream*                 m_in{nullptr};
    std::shared_ptr<std::istream> m_my_in;
    size_t                        m_curLineNum{0};
    bool                          m_filter_MATLAB_comments{true};
    bool                          m_filter_C_comments{true};
    bool                          m_filter_SH_comments{true};

};  // end of CTextFileLinesParser
}  // namespace openbeam
