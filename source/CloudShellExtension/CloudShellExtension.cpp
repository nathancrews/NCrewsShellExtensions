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

#include "CloudShellExtension.h"
#include "CloudShellExtensionDll.h"
#include "CloudMenuGUID.h"
#include "CloudClassFactory.h"
#include "Renderers/RenderToImageCommon.h"

void cloud_print_fcn(const std::string& logString)
{
#ifdef NO_LOGGING
    return;
#endif // NO_LOGGING

    std::filesystem::path logFilePath = std::filesystem::temp_directory_path();

    logFilePath.replace_filename("CloudShellExtension");
    logFilePath.replace_extension("log");

    std::fstream fs;
    fs.open(logFilePath, std::fstream::out | std::fstream::app);

    fs << logString;
    fs << "\n";

    fs.flush();
    fs.close();
}

#define CHECK_HRESULT(res) (if (res != ERROR_SUCCESS) return E_UNEXPECTED;)


// Standard DLL functions
BOOL DllMain(HINSTANCE hInstance, DWORD dwReason, void*)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        g_hinst = hInstance;
        DisableThreadLibraryCalls(hInstance);

        HRESULT comInitStat = CoInitializeEx(0, COINIT_APARTMENTTHREADED);

        DWORD strSize = MAX_PATH;
        GetModuleFileNameW(hInstance, g_DllModelName, strSize);

        std::filesystem::path appPath = g_DllModelName;
        g_AppPath = appPath.remove_filename();

        open3d::utility::Logger::GetInstance().SetPrintFunction(cloud_print_fcn);

        WCHAR appdata[MAX_PATH] = { 0 };
        SHGetFolderPathW(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, appdata);
        g_AppDataPath = appdata;

        g_AppDataPath.concat(L"\\");
        g_AppDataPath.concat(g_ProducerName);
        g_AppDataPath.concat(L"\\");
        g_AppDataPath.concat(g_AppDirectoryName);
        g_AppDataPath.concat(L"\\");

        utility::LogInfo("appdata = {}", g_AppDataPath.string());

    }
    return TRUE;
}

HRESULT DllCanUnloadNow(void)
{
    CHAR ModuleRefCountStr[MAX_PATH] = { 0 };
    sprintf(ModuleRefCountStr, "%d", g_DllModuleRefCount);

    std::string refReport = "g_DllModuleRefCount = " + std::string(ModuleRefCountStr);

    utility::LogInfo("DllCanUnloadNow called...");
    utility::LogInfo(refReport.c_str());

    if (g_DllModuleRefCount <= 0)
    {
        utility::LogInfo("DllCanUnloadNow calling GdiplusShutdown and unloading.");

        utility::Logger::GetInstance().ResetPrintFunction();

        return S_OK;
    }

    return S_FALSE;
}

void DllAddRef(void)
{
    InterlockedIncrement(&g_DllModuleRefCount);
}

void DllRelease(void)
{
    InterlockedDecrement(&g_DllModuleRefCount);
}

HRESULT DllGetClassObject(_In_ REFCLSID clsid, _In_ REFIID riid, _Outptr_ LPVOID* ppv)
{
    if (!ppv)
    {
        return E_INVALIDARG;
    }

    HRESULT res = E_UNEXPECTED;

    if (IsEqualCLSID(clsid, CloudMenuGUID))
    {
        CloudClassFactory* nCF = new CloudClassFactory();
        if (nCF)
        {
            res = nCF->QueryInterface(riid, ppv);
            nCF->Release();
        }
    }

    return res;
}

HRESULT DllRegisterServer()
{
    HKEY hkey;
    DWORD lpDisp;
    HRESULT retStatus = E_UNEXPECTED;

    wchar_t* menuExtGUID = nullptr;
    DWORD res = StringFromCLSID(CloudMenuGUID, &menuExtGUID);
    std::wstring lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(menuExtGUID);

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_CREATE_SUB_KEY, NULL, &hkey, &lpDisp);

    wchar_t dllName[MAX_PATH] = { 0 };
    GetModuleFileName(g_hinst, dllName, MAX_PATH);

    lpSubKey = L"InProcServer32";
    retStatus = RegCreateKeyEx(hkey, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)dllName, (DWORD)(std::wstring(dllName).size() + 1U) * 2U);

    std::wstring lpKeyValue = L"Apartment";

    retStatus = RegSetValueEx(hkey, L"ThreadingModel", 0, REG_SZ, (BYTE*)lpKeyValue.c_str(), (DWORD)((lpKeyValue.size() + 1U) * 2U));

    RegCloseKey(hkey);


    //**************************************************************************************************************
    // Register for file types

    lpSubKey = L".las\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CLASSES_ROOT, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.las\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);

    RegCloseKey(hkey);


    lpSubKey = L".laz\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CLASSES_ROOT, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.laz\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);

    RegCloseKey(hkey);

    /*
    lpSubKey = L"Software\\Classes\\.pcd\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.ply\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);


    lpSubKey = L"Software\\Classes\\.pts\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.xyz\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (retStatus != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);*/

    //**************************************************************************************************************
    // For Directories
    lpSubKey = L"Software\\Classes\\Directory\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";

    retStatus = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);

    retStatus = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);

    RegCloseKey(hkey);

    //
    //**************************************************************************************************************
    // Put extension on the approved list - try both HKLM and HKCU for Windows 11 compatibility
    lpSubKey = L"SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Shell Extensions\\Approved";

    // Try HKEY_LOCAL_MACHINE first (preferred for Windows 11)
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_LOCAL_MACHINE, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        RegSetValueEx(hkey, std::wstring(menuExtGUID).c_str(), 0, REG_SZ, (BYTE*)L"Point Cloud Shell Extension", (DWORD)(wcslen(L"Point Cloud Shell Extension") + 1) * 2U);
        RegCloseKey(hkey);
    }
    // Fallback to HKEY_CURRENT_USER
    else if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegSetValueEx(hkey, std::wstring(menuExtGUID).c_str(), 0, REG_SZ, (BYTE*)L"Point Cloud Shell Extension", (DWORD)(wcslen(L"Point Cloud Shell Extension") + 1) * 2U);
        RegCloseKey(hkey);
    }
    //**************************************************************************************************************


    // Delete laz_auto_file entries, they mess up the menu commands displaying properly
    lpSubKey = L"laz_auto_file";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CLASSES_ROOT, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CLASSES_ROOT, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\laz_auto_file";

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_LOCAL_MACHINE, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_LOCAL_MACHINE, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_CONFIG, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CURRENT_CONFIG, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

// Delete las_auto_file entries, they mess up the menu commands displaying properly
    lpSubKey = L"las_auto_file";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CLASSES_ROOT, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CLASSES_ROOT, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\las_auto_file";

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_LOCAL_MACHINE, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_LOCAL_MACHINE, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_CONFIG, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        retStatus = RegDeleteKey(HKEY_CURRENT_CONFIG, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    //*****************************************************************************************
    // Windows 11 specific: Enhanced shell notifications and cache management
    
    // Add enhanced file association entries for Windows 11
    lpSubKey = L"Software\\Classes\\.las";
    if (ERROR_SUCCESS == RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp))
    {
        DWORD disableCache = 1;
        RegSetValueEx(hkey, L"DisableThumbnailCache", 0, REG_DWORD, (BYTE*)&disableCache, sizeof(DWORD));
        RegCloseKey(hkey);
    }
    
    lpSubKey = L"Software\\Classes\\.laz";
    if (ERROR_SUCCESS == RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp))
    {
        DWORD disableCache = 1;
        RegSetValueEx(hkey, L"DisableThumbnailCache", 0, REG_DWORD, (BYTE*)&disableCache, sizeof(DWORD));
        RegCloseKey(hkey);
    }

    SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, NULL, NULL);
    // Additional notification for Windows 11 compatibility
    SHChangeNotify(SHCNE_UPDATEIMAGE, SHCNF_DWORD | SHCNF_FLUSH, NULL, NULL);

    return S_OK;
}

HRESULT DllUnregisterServer()
{
    HKEY hkey;
    wchar_t* tempStr = nullptr;
    DWORD res = StringFromCLSID(CloudMenuGUID, &tempStr);

    std::wstring lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(tempStr) + L"\\InprocServer32";

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());
        if (res != ERROR_SUCCESS)
        {
            return E_UNEXPECTED;
        }

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(tempStr);

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());
        if (res != ERROR_SUCCESS)
        {
            return E_UNEXPECTED;
        }

        RegCloseKey(hkey);
    }


    lpSubKey = L"Software\\Classes\\Directory\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\.las\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\.laz\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    /*
    lpSubKey = L"Software\\Classes\\.pcd\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\.ply\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\.xyz\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }

    lpSubKey = L"Software\\Classes\\.pts\\ShellEx\\ContextMenuHandlers\\CloudShellExtension";
    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegDeleteKey(HKEY_CURRENT_USER, lpSubKey.c_str());

        RegCloseKey(hkey);
    }
*/

    SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, NULL, NULL);

    return S_OK;
}