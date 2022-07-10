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

#include "localization.h"

#include <iostream>

// Needed for SetConsoleOutputCP()
#ifdef _WIN32
#define WINDOWS_LEAN_AND_MEAN
#include <io.h>
#include <windows.h>
#endif

using namespace openbeam::localization;

/** Singleton pattern to make sure initialization before first usage in any
 * case. */
class TLangSystem
{
   public:
    TLanguage selectedLang;

    inline static TLangSystem& getInstance()
    {
        static TLangSystem inst;
        return inst;
    }

   private:
    TLangSystem() : selectedLang(LANG_EN)
    {
        // In Windows, make sure the console uses UTF-8:
        //  http://stackoverflow.com/questions/1660492/utf-8-output-on-windows-xp-console
        // (But still it doesn't work...)
#ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);
#endif
    }
};

struct TLangData
{
    TLanguage    id;  //!< Enum value
    const char*  lang_name;  //!< ISO name of language
    const char** string_table;  //!< Strings tranlation tables
};

// This MUST BE in the same language order as declared in languages.h
const TLangData lang_data[NUMBER_OF_LANGUAGES] = {
    {LANG_EN, "en", strs_en}, {LANG_ES, "es", strs_es}};

/** Selects the language from its 2 letter ISO 639-1 code
 *  ("en","es",...). \return false on unsupported language. */
bool openbeam::localization::selectLanguage(const TLanguage langID)
{
    if (langID >= 0 && langID < NUMBER_OF_LANGUAGES)
    {
        TLangSystem::getInstance().selectedLang = langID;
        return true;
    }
    else
        return false;
}

bool openbeam::localization::selectLanguage(const std::string& langID)
{
    // std::cout << "[openbeam::localization] Selecting language: " << langID <<
    // std::endl;
    for (int i = 0; i < sizeof(lang_data) / sizeof(lang_data[0]); i++)
    {
        if (std::string(lang_data[i].lang_name) == langID)
        {
            TLangSystem::getInstance().selectedLang = lang_data[i].id;
            return true;
        }
    }
    return false;
}

/** The main translation function, takes a string code and return its
 * translation for the currently selected language */
const char* openbeam::localization::_t(const TStringID strID)
{
    TLangSystem& ls = TLangSystem::getInstance();

    if (ls.selectedLang < NUMBER_OF_LANGUAGES && strID < STR_NUMBER_OF_IDS)
        return lang_data[ls.selectedLang].string_table[strID];
    else
        return "<text not found>";
}
