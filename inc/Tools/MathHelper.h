////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan C. Crews IV
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////////


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
