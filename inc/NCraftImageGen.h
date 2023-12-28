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