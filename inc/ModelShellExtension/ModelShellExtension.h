////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan Crews, NCrews Software
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
#include "priv.h"
#include <filesystem>
#include <iostream>
#include <fstream>

STDAPI_(BOOL) DllMain(HINSTANCE hInstance, DWORD dwReason, void*);
STDAPI DllCanUnloadNow(void);
void __stdcall DllAddRef(void);
void __stdcall DllRelease(void);
STDAPI DllGetClassObject(_In_ REFCLSID clsid, _In_ REFIID riid, _Outptr_ LPVOID* ppv);
HRESULT __stdcall  DllRegisterServer();
HRESULT __stdcall  DllUnregisterServer();

extern std::wstring g_AppName;
extern long g_DllModuleRefCount;
extern TCHAR g_DllModelName[MAX_PATH];
extern std::filesystem::path g_AppPath;
extern std::wstring g_appUserModelID;
extern std::wstring g_ProducerName;
extern std::wstring g_AppDirectoryName;
extern std::filesystem::path g_AppDataPath;
extern std::filesystem::path g_SettingsFileName;
extern void model_print_fcn(const std::string& logString);