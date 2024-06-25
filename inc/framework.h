#pragma once

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define _CRT_FUNCTIONS_REQUIRED 1

#include <windows.h>
#include "targetver.h"

#endif

#include <cstdlib>
#include <iostream>
#include <numeric>