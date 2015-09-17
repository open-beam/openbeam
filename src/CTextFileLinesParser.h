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


#pragma once

#include <openbeam/types.h>

namespace openbeam
{
        /** A class for parsing text files, returning each non-empty and non-comment line, along its line number.
          *  Lines are strip out of leading and trailing whitespaces.
          *  By default, lines starting with either "#", "//" or "%" are skipped (comment lines),
          *   unless this behavior is explicitly disabled with  \a enableCommentFilters
          *
          * \note This class is copied from the MRPT project.
          */
        class CTextFileLinesParser
        {
        public:
            /** Default constructor; should call \a open() at some moment later. */
            CTextFileLinesParser() : m_in(NULL), m_own_in(true),m_curLineNum(0), m_filter_MATLAB_comments(true), m_filter_C_comments(true), m_filter_SH_comments(true) { }

            ~CTextFileLinesParser() { close(); }

            /** Close the file (no need to call it normally, the file is closed upon destruction) */
            void close()
            {
            	if (m_in && m_own_in)
            	{
					delete m_in;
					m_in = NULL;
            	}
            }

            /** Open a file \return true on success */
            bool open(const std::string &fil)
            {
                m_curLineNum = 0;
                m_fileName = fil;
                close();
                std::ifstream * in_fil = new std::ifstream();
                m_in = in_fil;
                m_own_in = true;
                m_in->clear();
                in_fil->open(fil.c_str());
                return in_fil->is_open();
            }

            void setInputStream(std::istream *is, bool become_owner = false)
            {
            	close();
            	m_in = is;
            	m_own_in = become_owner;
            }

            /** Reset the read pointer to the beginning of the file */
            void rewind()
            {
                m_curLineNum = 0;
                if (m_in)
                {
					m_in->clear();
					m_in->seekg(0);
                }
            }

            /** Reads from the file and return the next (non-comment) line, as a std::string
              * \return false on EOF.
              */
            inline bool getNextLine(std::string &out_str)
            {
                std::istringstream buf;
                if (getNextLine(buf))
                {
                    out_str = buf.str();
                    return true;
                }
                else
                {
                    out_str.clear();
                    return false;
                }
            }

            /** Reads from the file and stores the next (non-comment) line into the given stream buffer.
              * \return false on EOF.
              */
            bool getNextLine( std::istringstream &buf )
            {
            	if (!m_in) return false;

                while (!m_in->fail())
                {
                    std::string lin;
                    std::getline(*m_in,lin);
                    m_curLineNum++;
                    lin = trim(lin);
                    if (lin.empty()) continue; // Ignore empty lines.
                    // Ignore comments lines, starting with "#" or "//".
                    if ( (m_filter_SH_comments && strStarts(lin,"#"))
                      || (m_filter_C_comments  && strStarts(lin,"//"))
                      || (m_filter_MATLAB_comments && strStarts(lin,"%")) )
                        continue;
                    // Parse the line as a string stream:

                    // Look for potential comments in the middle of the line and strip them:
					for (;;)
					{
						if (m_filter_SH_comments) {
							const size_t idxComm = lin.find("#");
							if (idxComm!=std::string::npos)  {
								lin=lin.substr(0,idxComm);
								continue;
							}
						}
						if (m_filter_C_comments) {
							const size_t idxComm = lin.find("//");
							if (idxComm!=std::string::npos)  {
								lin=lin.substr(0,idxComm);
								continue;
							}
						}
						if (m_filter_MATLAB_comments) {
							const size_t idxComm = lin.find("%");
							if (idxComm!=std::string::npos)  {
								lin=lin.substr(0,idxComm);
								continue;
							}
						}
						// No more comments, go on:
						break;
					}
                    buf.str(lin);
                    buf.clear();
                    return true;
                };
                return false;
            }

            /** Return the line number of the last line returned with \a getNextLine */
            inline size_t getCurrentLineNumber() const { return m_curLineNum; }

            /** Enable/disable filtering of lines starting with "%", "//" or "#", respectively. */
            inline void enableCommentFilters(
                bool filter_MATLAB_comments,
                bool filter_C_comments,
                bool filter_SH_comments
                )
            {
                m_filter_MATLAB_comments = filter_MATLAB_comments;
                m_filter_C_comments = filter_C_comments;
                m_filter_SH_comments = filter_SH_comments;
            }

        private:
            std::string   m_fileName;
            std::istream  *m_in;
            bool          m_own_in;
            size_t        m_curLineNum;
            bool		  m_filter_MATLAB_comments;
            bool		  m_filter_C_comments;
            bool		  m_filter_SH_comments;


			// Aux. functions (taken from MRPT):
			static std::string trim(const std::string &str)
			{
				if (str.empty())
					return std::string();
				else
				{
					size_t s = str.find_first_not_of(" \t");
					size_t e = str.find_last_not_of(" \t");
					if (s==std::string::npos || e==std::string::npos)
							return std::string();
					else	return str.substr( s, e-s+1);
				}
			}

			/** Return true if "str" starts with "subStr" (case sensitive)  \sa strStartsI  */
			static bool strStarts(const std::string &s1, const std::string &s2)
			{
				return ! ::strncmp(s1.c_str(),s2.c_str(),s2.size()); // if s1 is shorter it's not a problem
			}

        };  // end of CTextFileLinesParser

} // end of namespace

