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
extern std::wstring g_appUserModelID;
extern long g_DllModuleRefCount;
extern TCHAR g_DllModelName[MAX_PATH];
extern std::filesystem::path g_AppPath;
extern std::wstring g_ProducerName;
extern std::wstring g_AppDirectoryName;
extern std::filesystem::path g_SettingsFileName;
extern std::vector<std::string> g_Settings_ImageImageSizes;
extern std::filesystem::path g_AppDataPath;

extern void cloud_print_fcn(const std::string& logString);