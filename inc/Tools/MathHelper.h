#pragma once
#include <cfloat>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4244)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif

class MathHelper
{
public:

    static inline bool IsFuzzyEqual(const double lhs, const double rhs, const double tol = 0.001)
    {
        if (std::abs(lhs - rhs) < tol)
        {
            return true;
        }
        return false;
    }

    static inline bool IsFuzzyZero(const float lhs, const double tol = 0.0001f)
    {
        if (std::abs(lhs) < tol)
        {
            return true;
        }
        return false;
    }

    static inline double IFT2M		=   0.3048;
    static inline double USFT2M	    =	0.3048006096012192;
    static inline double PI         =   3.1415926535897932384626433832795;

};

#ifdef _MSC_VER
#pragma warning(pop)
#endif
