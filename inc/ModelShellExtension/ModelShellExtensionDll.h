#pragma once

// Include this file ONCE in DllMain

// Handle the the DLL's module
HINSTANCE g_hinst = NULL;
long g_DllModuleRefCount = 0;
TCHAR g_DllModelName[MAX_PATH] = { 0 };
std::filesystem::path g_AppPath;
std::wstring g_AppName = L"Model Shell Extension";
std::wstring g_RegAppName = L"ModelShellExtension";
ULONG_PTR g_gpToken;

