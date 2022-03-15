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

#include <openbeam/config.h>

#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

// Useful macros ---------------------
#if defined(__BORLANDC__)
#define _CURRENT_FUNC_ __FUNC__
#else
#define _CURRENT_FUNC_ __FUNCTION__
#endif

#if defined(_WIN32) || defined(_WIN32_) || defined(WIN32) || defined(_WIN64)
#define OB_OS_WINDOWS
#else
#define OB_OS_UNIX
#endif

#define OBASSERT(_F)                               \
    {                                              \
        if (!(_F))                                 \
        {                                          \
            std::stringstream s;                   \
            s << _CURRENT_FUNC_ << ":" << __LINE__ \
              << ": Assert failed: " << #_F;       \
            throw std::runtime_error(s.str());     \
        }                                          \
    }

#ifdef _DEBUG
#define OBASSERT_DEBUG(_F) OBASSERT(_F)
#else
#define OBASSERT_DEBUG(_F)
#endif

/** Usage: OB_MESSAGE(num) << "blah" << x << endl;
 *  Where num is the minimum OpenBeam verbose level for the message to actually
 * be emitted to std::cout. \sa openbeam::setVerbosityLevel
 */
#define OB_MESSAGE(VERBOSE_LEVEL)                       \
    if (openbeam::getVerbosityLevel() >= VERBOSE_LEVEL) \
    std::cout << "[" << _CURRENT_FUNC_ << "] "

// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#define OB_printf_format_check(_FMT_, _VARARGS_) \
    __attribute__((__format__(__printf__, _FMT_, _VARARGS_)))
#else
#define OB_printf_format_check(_FMT_, _VARARGS_)
#endif

// Declare compiler messages:
#if defined(_MSC_VER)
#define OB_DO_PRAGMA(x) __pragma(x)
#define __STR2__(x) #x
#define __STR1__(x) __STR2__(x)
#define __MSVCLOC__ __FILE__ "("__STR1__(__LINE__) ") : "
#define OB_MSG_PRAGMA(_msg) OB_DO_PRAGMA(message(__MSVCLOC__ _msg))
#elif defined(__GNUC__)
#define OB_DO_PRAGMA(x) _Pragma(#x)
#define OB_MSG_PRAGMA(_msg) OB_DO_PRAGMA(message(_msg))
#else
#define OB_DO_PRAGMA(x)
#define OB_MSG_PRAGMA(_msg)
#endif

#define OB_WARNING(x) OB_MSG_PRAGMA("Warning: " #x)
#define OB_TODO(x) OB_MSG_PRAGMA("TODO: " #x)

#define UNINITIALIZED_VALUE (std::numeric_limits<num_t>::max())

namespace openbeam
{
inline num_t DEG2RAD(const num_t x) { return x * M_PI / num_t(180.0); }
inline num_t RAD2DEG(const num_t x) { return x * num_t(180.0) / M_PI; }
template <typename T>
inline T square(T x)
{
    return x * x;
}

class CTimeLogger;
extern CTimeLogger timelog;  //!< A global timelogger for openbeam

/** @name Types
    @{ */

/** A case-insensitive comparator struct for usage within STL containers, eg:
 * map<string,string,ci_less> */
struct ci_less : std::binary_function<std::string, std::string, bool>
{
    // case-independent (ci) compare_less binary function
    struct nocase_compare : public std::binary_function<char, char, bool>
    {
        bool operator()(const char c1, const char c2) const
        {
            return tolower(c1) < tolower(c2);
        }
    };
    bool operator()(const std::string& s1, const std::string& s2) const
    {
        return std::lexicographical_compare(
            s1.begin(), s1.end(), s2.begin(), s2.end(), nocase_compare());
    }
};  // end of ci_less

typedef Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic> TDynMatrix;
typedef Eigen::Matrix<num_t, 6, 6>                           TMatrix66;
typedef Eigen::Matrix<num_t, 3, 3>                           TMatrix33;
typedef Eigen::Matrix<num_t, 6, 1>                           TVector6;
typedef Eigen::Matrix<num_t, 3, 1>                           TVector3;

typedef std::map<std::string, std::string, ci_less>
    TParamSet;  //!< A set of parameters, indexed by name (case insensitive)

typedef std::vector<std::string> vector_string;

typedef std::map<std::string, num_t, ci_less> map_string2num_ci;

/** Stress tensor for a 3D element */
struct TFaceStress
{
    TFaceStress() { N = Vy = Vz = Mx = My = Mz = 0; }

    num_t N;  //!< Axial
    num_t Vy;  //!< Shear Y
    num_t Vz;  //!< Shear Z
    num_t Mx;  //!< Moment X
    num_t My;  //!< Moment Y
    num_t Mz;  //!< Moment Z

    TFaceStress& operator+=(const TFaceStress& o)
    {
        N += o.N;
        Vy += o.Vy;
        Vz += o.Vz;
        Mx += o.Mx;
        My += o.My;
        Mz += o.Mz;
        return *this;
    }

    template <class MAT>
    TFaceStress& operator+=(const Eigen::MatrixBase<MAT>& o)
    {
        OBASSERT_DEBUG(o.size() == 6)
        N += o[0];
        Vy += o[1];
        Vz += o[2];
        Mx += o[3];
        My += o[4];
        Mz += o[5];
        return *this;
    }
};

typedef std::vector<TFaceStress>
    TElementStress;  //!< One entry per element "face".

struct TEvaluationContext
{
    TEvaluationContext()
        : err_msgs(nullptr), warn_msgs(nullptr), lin_num(0), lin(nullptr)
    {
    }

    /** Return false only if there was an error if there's not "errMsg".
     * Should only process the case of returning true and do nothing else on
     * return false to handle the error. */
    bool parser_evaluate_expression(
        const std::string& sVarVal, num_t& val) const;

    map_string2num_ci user_vars;
    vector_string*    err_msgs;
    vector_string*    warn_msgs;
    unsigned int      lin_num;
    std::string*      lin;
};

struct TRotation3D
{
    inline TRotation3D() : rot(TMatrix33::Identity()), m_is_pure_identity(true)
    {
    }
    TRotation3D(const num_t ang_x, const num_t ang_y, const num_t ang_z);

    inline const TMatrix33& getRot() const { return rot; }
    inline void             setRot(const TMatrix33& r)
    {
        rot                = r;
        m_is_pure_identity = false;
    }

    static void matrix2angles(
        const TMatrix33& R, num_t& ang_x, num_t& ang_y, num_t& ang_z);

    bool isIdentity()
        const;  //!< Test for whether this 3x3 matrix is EXACTLY the identity

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    TMatrix33 rot;
    bool      m_is_pure_identity;
};

struct TPoint3D
{
    num_t coords[3];

    TPoint3D() {}
    TPoint3D(const num_t x, const num_t y, const num_t z)
    {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    }
    inline num_t norm_squared() const
    {
        return coords[0] * coords[0] + coords[1] * coords[1] +
               coords[2] * coords[2];
    }
    inline num_t norm() const { return std::sqrt(norm_squared()); }
};

struct TRotationTrans3D
{
    inline TRotationTrans3D() : t(0, 0, 0), r() {}
    inline TRotationTrans3D(
        num_t x, num_t y, num_t z, num_t ang_x, num_t ang_y, num_t ang_z)
        : t(x, y, z), r(ang_x, ang_y, ang_z)
    {
    }

    TPoint3D    t;  //!< Translation vector in 3D
    TRotation3D r;  //!< Rotation in 3D

    inline num_t distanceTo(const TRotationTrans3D& o) const
    {
        return std::sqrt(
            openbeam::square(t.coords[0] - o.t.coords[0]) +
            openbeam::square(t.coords[1] - o.t.coords[1]) +
            openbeam::square(t.coords[2] - o.t.coords[2]));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @} */

/** Changes the overall library verbosity level: 0=quiet, 1=interesting
 * messages, 2=verbose. \sa getVerbosityLevel
 */
void setVerbosityLevel(int verbose_level);

/** Get the current verbosity level of the library.
 * \sa setVerbosityLevel
 */
int getVerbosityLevel();

/** Return true if the two strings are equal (case insensitive)  */
bool strCmpI(const std::string& s1, const std::string& s2);

/** Returns the numeric representation of a string, or raises an exception if
 * it's not a valid number */
num_t str2num(const std::string& s);

/** A sprintf-like function for std::string */
std::string format(const char* fmt, ...) OB_printf_format_check(1, 2);
;

/** An OS-independent method for tokenizing a string.
 * The extra parameter "context" must be a pointer to a "char*" variable, which
 * needs no initialization and is used to save information between calls to
 * strtok. \sa tokenize */
char* strtok(char* str, const char* strDelimit, char** context);

/** Tokenizes a string according to a set of delimiting characters.
    * Example:
    * \code
    vector_string	tokens;
    tokenize( " - Pepe-Er  Muo"," -",tokens);
    * \endcode
    *
    *  Will generate 3 tokens:
    *		- "Pepe"
    *		- "Er"
    *		- "Muo"
    */
void tokenize(
    const std::string& inString, const std::string& inDelimiters,
    vector_string& outTokens, bool do_trim_all_parts = true);

/**  Removes trailing and leading whitespaces. */
std::string trim(const std::string& str);

/** Helper types for STL containers with Eigen memory allocators. */
template <class TYPE1, class TYPE2 = TYPE1>
struct aligned_containers
{
    typedef std::pair<TYPE1, TYPE2>                             pair_t;
    typedef std::vector<TYPE1, Eigen::aligned_allocator<TYPE1>> vector_t;
    typedef std::deque<TYPE1, Eigen::aligned_allocator<TYPE1>>  deque_t;
    typedef std::map<
        TYPE1, TYPE2, std::less<TYPE1>,
        Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2>>>
        map_t;
    typedef std::multimap<
        TYPE1, TYPE2, std::less<TYPE1>,
        Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2>>>
        multimap_t;
};

/** Helper function to erase a STL container of pointers freeing all the object
 */
template <typename CONTAINER>
void free_container(CONTAINER& c)
{
    for (typename CONTAINER::iterator it = c.begin(); it != c.end(); ++it)
        delete (*it);
    c.clear();
}

/** Helper function to erase a STL container of pointers freeing all the object
 */
template <typename CONTAINER>
void free_assoc_container(CONTAINER& c)
{
    for (typename CONTAINER::iterator it = c.begin(); it != c.end(); ++it)
        delete it->second;
    c.clear();
}

// --------  start of array STL wrapper -----------------------
template <typename T, std::size_t N>
class array
{
   public:
    T elems[N];  // fixed-size array of elements of type T
   public:
    // type definitions
    typedef T              value_type;
    typedef T*             iterator;
    typedef const T*       const_iterator;
    typedef T&             reference;
    typedef const T&       const_reference;
    typedef std::size_t    size_type;
    typedef std::ptrdiff_t difference_type;

    // iterator support
    iterator       begin() { return elems; }
    const_iterator begin() const { return elems; }
    iterator       end() { return elems + N; }
    const_iterator end() const { return elems + N; }

    // reverse iterator support
    typedef std::reverse_iterator<iterator>       reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    reverse_iterator       rbegin() { return reverse_iterator(end()); }
    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }
    reverse_iterator       rend() { return reverse_iterator(begin()); }
    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    // operator[]
    reference       operator[](size_type i) { return elems[i]; }
    const_reference operator[](size_type i) const { return elems[i]; }

    // at() with range check
    reference at(size_type i)
    {
        rangecheck(i);
        return elems[i];
    }
    const_reference at(size_type i) const
    {
        rangecheck(i);
        return elems[i];
    }

    // front() and back()
    reference       front() { return elems[0]; }
    const_reference front() const { return elems[0]; }
    reference       back() { return elems[N - 1]; }
    const_reference back() const { return elems[N - 1]; }

    // size is constant
    static size_type size() { return N; }
    static bool      empty() { return false; }
    static size_type max_size() { return N; }
    enum
    {
        static_size = N
    };

    // swap (note: linear complexity in N, constant for given instantiation)
    void swap(array<T, N>& y) { std::swap_ranges(begin(), end(), y.begin()); }

    // direct access to data (read-only)
    const T* data() const { return elems; }

    // use array as C array (direct read/write access to data)
    T* data() { return elems; }

    // assignment with type conversion
    template <typename T2>
    array<T, N>& operator=(const array<T2, N>& rhs)
    {
        std::copy(rhs.begin(), rhs.end(), begin());
        return *this;
    }

    // assign one value to all elements
    void assign(const T& value) { std::fill_n(begin(), size(), value); }

   private:
    // check range (may be private because it is static)
    static void rangecheck(size_type i)
    {
        if (i >= size())
        { throw std::out_of_range("array<>: index out of range"); }
    }
};

// partial specialization for arrays of size 0
template <typename T>
class array<T, 0>
{
   public:
    char c;  // to ensure different array intances return unique values for
             // begin/end

   public:
    // type definitions
    typedef T              value_type;
    typedef T*             iterator;
    typedef const T*       const_iterator;
    typedef T&             reference;
    typedef const T&       const_reference;
    typedef std::size_t    size_type;
    typedef std::ptrdiff_t difference_type;

    // iterator support
    iterator       begin() { return reinterpret_cast<iterator>(&c); }
    const_iterator begin() const
    {
        return reinterpret_cast<const_iterator>(&c);
    }
    iterator       end() { return reinterpret_cast<iterator>(&c); }
    const_iterator end() const { return reinterpret_cast<const_iterator>(&c); }

    // reverse iterator support
    typedef std::reverse_iterator<iterator>       reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    reverse_iterator       rbegin() { return reverse_iterator(end()); }
    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }
    reverse_iterator       rend() { return reverse_iterator(begin()); }
    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    // at() with range check
    reference at(size_type i)
    {
        throw std::out_of_range("array<0>: index out of range");
    }
    const_reference at(size_type i) const
    {
        throw std::out_of_range("<0>: index out of range");
    }

    // size is constant
    static size_type size() { return 0; }
    static bool      empty() { return true; }
    static size_type max_size() { return 0; }
    enum
    {
        static_size = 0
    };

    // swap
    void swap(array<T, 0>& y)
    {
        //  could swap value of c, but value is not part of documented array
        //  state
    }

    // direct access to data
    const T* data() const { return nullptr; }
    T*       data() { return nullptr; }

    // assignment with type conversion
    template <typename T2>
    array<T, 0>& operator=(const array<T2, 0>& rhs)
    {
        return *this;
    }

    //  Calling these operations are undefined behaviour for 0-size arrays,
    //  but Library TR1 requires their presence.
    // operator[]
    reference       operator[](size_type i) { makes_no_sense(); }
    const_reference operator[](size_type i) const { makes_no_sense(); }

    // front() and back()
    reference       front() { makes_no_sense(); }
    const_reference front() const { makes_no_sense(); }
    reference       back() { makes_no_sense(); }
    const_reference back() const { makes_no_sense(); }

   private:
    // helper for operations that have undefined behaviour for 0-size arrays,
    //  assert( false ); added to make lack of support clear
    static void makes_no_sense()
    {
        assert(true);
        throw std::out_of_range("array<0>: index out of range");
    }
};

// comparisons
template <class T, std::size_t N>
bool operator==(const array<T, N>& x, const array<T, N>& y)
{
    return std::equal(x.begin(), x.end(), y.begin());
}
template <class T, std::size_t N>
bool operator<(const array<T, N>& x, const array<T, N>& y)
{
    return std::lexicographical_compare(x.begin(), x.end(), y.begin(), y.end());
}
template <class T, std::size_t N>
bool operator!=(const array<T, N>& x, const array<T, N>& y)
{
    return !(x == y);
}
template <class T, std::size_t N>
bool operator>(const array<T, N>& x, const array<T, N>& y)
{
    return y < x;
}
template <class T, std::size_t N>
bool operator<=(const array<T, N>& x, const array<T, N>& y)
{
    return !(y < x);
}
template <class T, std::size_t N>
bool operator>=(const array<T, N>& x, const array<T, N>& y)
{
    return !(x < y);
}
// global swap()
template <class T, std::size_t N>
inline void swap(array<T, N>& x, array<T, N>& y)
{
    x.swap(y);
}

// ----------- end of array STL wrapper -----------------------

typedef array<num_t, 6> array6;

/** By creating an object of this class, all the output to std::cout (and
 * std::cerr) will be redirected to a text file, and optionally also shown on
 * the console. Class copied from MRPT (http://www.mrpt.org/)
 */
class CConsoleRedirector : public std::streambuf
{
   protected:
    std::ofstream   m_of;  //!< The text output file stream.
    std::streambuf* sbOld;  //!< The "old" std::cout
    std::streambuf* sbOld_cerr;  //!< The "old" std::cout
    bool            m_also_to_console;

   public:
    /** Constructor
     * \param out_file The file to create / append
     * \param also_to_console Whether to redirect data to file *and* also dump
     * data to the console as usual. \param append_file If set to false the file
     * will be truncated on open \param bufferSize It's recommended to buffer
     * the data instead of writing characters one by one. \param also_cerr
     * Whether to redirect the output to std::cerr in addition to std::cout.
     * \exception std::exception If the file cannot be opened.
     */
    CConsoleRedirector(
        const std::string& out_file, bool also_to_console = true,
        bool also_cerr = true, bool append_file = false, int bufferSize = 1000)
        : m_of(),
          sbOld(nullptr),
          sbOld_cerr(nullptr),
          m_also_to_console(also_to_console)
    {
        // Open the file:
        std::ios_base::openmode openMode =
            std::ios_base::binary | std::ios_base::out;
        if (append_file) openMode |= std::ios_base::app;
        m_of.open(out_file.c_str(), openMode);
        if (!m_of.is_open())
            throw std::runtime_error(
                format("Error opening file: %s", out_file.c_str()));

        if (bufferSize)
        {
            char* ptr = new char[bufferSize];
            setp(ptr, ptr + bufferSize);
        }
        else
            setp(0, 0);
        // Redirect:
        sbOld = std::cout.rdbuf();
        std::cout.rdbuf(this);
        if (also_cerr)
        {
            sbOld_cerr = std::cerr.rdbuf();
            std::cerr.rdbuf(this);
        }
    }
    virtual ~CConsoleRedirector()
    {
        sync();
        // Restore normal output:
        std::cout.rdbuf(sbOld);
        if (sbOld_cerr != nullptr) std::cerr.rdbuf(sbOld_cerr);
        if (pbase()) delete[] pbase();
    }

    void flush() { sync(); }

    virtual void writeString(const std::string& str)
    {
        if (m_also_to_console) sbOld->sputn(str.c_str(), str.size());
        m_of << str;
    }

   private:
    int overflow(int c)
    {
        sync();
        if (c != EOF)
        {
            if (pbase() == epptr())
            {
                std::string temp;
                temp += char(c);
                writeString(temp);
            }
            else
                sputc(c);
        }
        return 0;
    }

    int sync()
    {
        if (pbase() != pptr())
        {
            int         len = int(pptr() - pbase());
            std::string temp(pbase(), len);
            writeString(temp);
            setp(pbase(), epptr());
        }
        return 0;
    }
};

}  // namespace openbeam
