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

#include <openbeam/CTimeLogger.h>

using namespace std;
using namespace openbeam;

// A global timelogger for openbeam
CTimeLogger  openbeam::timelog; 


std::string rightPad(const std::string &str, const size_t total_len, bool truncate_if_larger)
{
	std::string r = str;
	if (r.size()<total_len || truncate_if_larger)
		r.resize(total_len,' ');
	return r;
}

/** This function implements formatting with the appropriate SI metric unit prefix:
  1e-12->'p', 1e-9->'n', 1e-6->'u', 1e-3->'m',
  1->'',
  1e3->'K', 1e6->'M', 1e9->'G', 1e12->'T'
 */
std::string unitsFormat(const double val,int nDecimalDigits, bool middle_space)
{
	char	prefix;
	double	mult;

	if (val>=1e12)
		{mult=1e-12; prefix='T';}
	else if (val>=1e9)
		{mult=1e-9; prefix='G';}
	else if (val>=1e6)
		{mult=1e-6; prefix='M';}
	else if (val>=1e3)
		{mult=1e-3; prefix='K';}
	else if (val>=1)
		{mult=1; prefix=' ';}
	else if (val>=1e-3)
		{mult=1e+3; prefix='m';}
	else if (val>=1e-6)
		{mult=1e+6; prefix='u';}
	else if (val>=1e-9)
		{mult=1e+9; prefix='n';}
	else
		{mult=1e+12; prefix='p';}

	return format(
		middle_space ? "%.*f %c" : "%.*f%c",
		nDecimalDigits,
		val*mult,
		prefix );
}


CTimeLogger::CTimeLogger(bool enabled) : m_tictac(), m_enabled(enabled)
{
	m_tictac.Tic();
}

CTimeLogger::~CTimeLogger()
{
	// Dump all stats:
    if (!m_data.empty()) // If logging is disabled, do nothing...
        dumpAllStats();
}

void CTimeLogger::clear(bool deep_clear)
{
	if (deep_clear)
		m_data.clear();
	else
	{
		for (map<string,TCallData>::iterator i=m_data.begin();i!=m_data.end();++i)
			i->second = TCallData();
	}
}

std::string  aux_format_string_multilines(const std::string &s, const size_t len)
{
	std::string ret;

	for (size_t p=0;p<s.size();p+=len)
	{
		ret+=rightPad(s.c_str()+p,len,true);
		if (p+len<s.size())
			ret+="\n";
	}
	return ret;
}

void CTimeLogger::getStats(std::map<std::string,TCallStats> &out_stats) const
{
	out_stats.clear();
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		TCallStats &cs = out_stats[i->first];
		cs.min_t   = i->second.min_t;
		cs.max_t   = i->second.max_t;
		cs.total_t = i->second.mean_t;
		cs.mean_t  = i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0;
		cs.n_calls = i->second.n_calls;
	}
}

std::string CTimeLogger::getStatsAsText(const size_t column_width)  const
{
	std::string s;

	s+="--------------------------- MRPT CTimeLogger report --------------------------\n";
	s+="           FUNCTION                         #CALLS  MIN.T  MEAN.T MAX.T TOTAL \n";
	s+="------------------------------------------------------------------------------\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		const string sMinT   = unitsFormat(i->second.min_t,1,false);
		const string sMaxT   = unitsFormat(i->second.max_t,1,false);
		const string sTotalT = unitsFormat(i->second.mean_t,1,false);
		const string sMeanT  = unitsFormat(i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,1,false);

		s+=format("%s %7u %6s%c %6s%c %6s%c %6s%c\n",
			aux_format_string_multilines(i->first,39).c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			sMinT.c_str(), i->second.has_time_units ? 's':' ',
			sMeanT.c_str(),i->second.has_time_units ? 's':' ',
			sMaxT.c_str(),i->second.has_time_units ? 's':' ',
			sTotalT.c_str(),i->second.has_time_units ? 's':' ' );
	}

	s+="---------------------- End of MRPT CTimeLogger report ------------------------\n";

	return s;
}

void CTimeLogger::saveToCSVFile(const std::string &csv_file)  const
{
	std::string s;
	s+="FUNCTION, #CALLS, MIN.T, MEAN.T, MAX.T, TOTAL.T\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		s+=format("\"%s\",\"%7u\",\"%e\",\"%e\",\"%e\",\"%e\"\n",
			i->first.c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			i->second.min_t,
			i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,
			i->second.max_t,
			i->second.mean_t );
	}
	std::ofstream f(csv_file.c_str());
	f << s; 
}

void CTimeLogger::dumpAllStats(const size_t  column_width) const
{
	string s = getStatsAsText(column_width);
	printf("\n%s\n", s.c_str() );
}

void CTimeLogger::do_enter(const char *func_name)
{
	const string  s = func_name;
	TCallData &d = m_data[s];

	d.n_calls++;
	d.open_calls.push(0);  // Dummy value, it'll be written below
	d.open_calls.top() = m_tictac.Tac(); // to avoid possible delays.
}

double CTimeLogger::do_leave(const char *func_name)
{
	const double tim = m_tictac.Tac();

	const string  s = func_name;
	TCallData &d = m_data[s];

	if (!d.open_calls.empty())
	{
		const double At = tim - d.open_calls.top();
		d.open_calls.pop();

		d.mean_t+=At;
		if (d.n_calls==1)
		{
			d.min_t= At;
			d.max_t= At;
		}
		else
		{
			d.min_t = std::min(d.min_t, At);
			d.max_t = std::max(d.max_t, At);
		}
		return At;
	}
	else return 0; // This shouldn't happen!
}

void CTimeLogger::registerUserMeasure(const char *event_name, const double value)
{
    if (!m_enabled) return;
	const string  s = event_name;
	TCallData &d = m_data[s];

	d.has_time_units = false;
	d.mean_t+=value;
	if (++d.n_calls==1)
	{
		d.min_t= value;
		d.max_t= value;
	}
	else
	{
		d.min_t = std::min(d.min_t, value);
		d.max_t = std::max(d.max_t, value);
	}
}

CTimeLogger::TCallData::TCallData() :
	n_calls	(0),
	min_t	(0),
	max_t	(0),
	mean_t	(0),
	has_time_units(true)
{
}

double CTimeLogger::getMeanTime(const std::string &name)  const
{
	map<string,TCallData>::const_iterator it = m_data.find(name);
	if (it==m_data.end())
		 return 0;
	else return it->second.n_calls ? it->second.mean_t/it->second.n_calls : 0;
}


CTimeLoggerEntry::CTimeLoggerEntry(CTimeLogger &logger, const char*section_name ) : m_logger(logger),m_section_name(section_name)
{
	m_logger.enter(m_section_name);
}
CTimeLoggerEntry::~CTimeLoggerEntry()
{
	m_logger.leave(m_section_name);
}
