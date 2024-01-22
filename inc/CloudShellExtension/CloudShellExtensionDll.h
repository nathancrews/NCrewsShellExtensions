#pragma once

// Include this file ONCE in DllMain

// Handle the the DLL's module
HINSTANCE g_hinst = NULL;
long g_DllModuleRefCount = 0;
TCHAR g_DllModelName[MAX_PATH] = { 0 };
std::filesystem::path g_AppPath;

std::wstring g_AppName = L"Point Cloud Shell Extension";
std::wstring g_appUserModelID = L"NCraft Image Generator";
std::wstring g_ProducerName = L"NCraft Software";
std::wstring g_AppDirectoryName = L"CloudShellExtension";
std::wstring g_RegAppName = L"CloudShellExtension";

ULONG_PTR g_gpToken;

std::filesystem::path g_AppDataPath;
std::filesystem::path g_SettingsFileName = L"settings.xml";


