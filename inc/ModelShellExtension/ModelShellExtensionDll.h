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

// Include this file ONCE in DllMain

// Handle the the DLL's module
HINSTANCE g_hinst = NULL;
long g_DllModuleRefCount = 0;
TCHAR g_DllModelName[MAX_PATH] = { 0 };
std::filesystem::path g_AppPath;
std::wstring g_AppName = L"GLTF Shell Extension";
std::wstring g_appUserModelID = L"NCrews Image Generator";
std::wstring g_ProducerName = L"NCrews Software";
std::wstring g_AppDirectoryName = L"GLTFShellExtension";
std::wstring g_RegAppName = L"GLTFShellExtension";
ULONG_PTR g_gpToken;

std::filesystem::path g_AppDataPath;
std::filesystem::path g_SettingsFileName = L"settings.xml";