﻿////////////////////////////////////////////////////////////////////////////////////
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
#include "NCraftImageGen.h"
#include "NCraftImageGenDll.h"
//#include "Renderers/RenderToImageCommon.h"

// Standard DLL functions
BOOL DllMain(HINSTANCE hInstance, DWORD dwReason, void*)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        g_hinst = hInstance;
        DisableThreadLibraryCalls(hInstance);

        DWORD strSize = MAX_PATH;
        GetModuleFileNameW(hInstance, g_DllModelName, strSize);

        //std::filesystem::path appPath = g_DllModelName;
        //g_AppPath = appPath.remove_filename();

        //open3d::utility::Logger::GetInstance().SetPrintFunction(print_fcn);

        //std::wstring appName = L"NCraft Image Generator";
        //std::wstring appUserModelID = L"NCraft Image Message";
 
        //GdiplusStartupInput gpStartupInput;
        //Gdiplus::Status mstat = GdiplusStartup(&g_gpToken, &gpStartupInput, NULL);
        //if (mstat != Gdiplus::Status::Ok)
        //{
        //    utility::LogInfo("Error: GdiplusStartup");
        //}

    }
    return TRUE;
}

#if 0
HRESULT DllCanUnloadNow(void)
{
    CHAR ModuleRefCountStr[MAX_PATH] = { 0 };
    sprintf(ModuleRefCountStr, "%d", g_DllModuleRefCount);

    std::string refReport = "g_DllModuleRefCount = " + std::string(ModuleRefCountStr);

    utility::LogInfo("DllCanUnloadNow called...");
    utility::LogInfo(refReport.c_str());

    if (g_DllModuleRefCount <= 0)
    {
        WinToastLib::WinToast::instance()->clear();

        utility::LogInfo("DllCanUnloadNow calling GdiplusShutdown and unloading.");
        GdiplusShutdown(g_gpToken);

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

    if (IsEqualCLSID(clsid, NCraftImageGenThumbnailGUID))
    {
        NCraftClassFactory* nCF = new NCraftClassFactory();
        if (nCF)
        {
            res = nCF->QueryInterface(riid, ppv);
            nCF->Release();
        }
    }

    if (IsEqualCLSID(clsid, NCraftImageGenMenuGUID))
    {
        NCraftClassFactory* nCF = new NCraftClassFactory();
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

    wchar_t* menuExtGUID = nullptr;
    DWORD res = StringFromCLSID(NCraftImageGenMenuGUID, &menuExtGUID);
    std::wstring lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(menuExtGUID);

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_CREATE_SUB_KEY, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    wchar_t dllName[MAX_PATH] = { 0 };
    GetModuleFileName(g_hinst, dllName, MAX_PATH);

    lpSubKey = L"InProcServer32";
    res = RegCreateKeyEx(hkey, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)dllName, (DWORD)(std::wstring(dllName).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    std::wstring lpKeyValue = L"Apartment";

    res = RegSetValueEx(hkey, L"ThreadingModel", 0, REG_SZ, (BYTE*)lpKeyValue.c_str(), (DWORD)((lpKeyValue.size() + 1U) * 2U));
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

//*****************************************************************************************
    // Thumbnail Extension

    wchar_t* thumbExtGUID = nullptr;

    res = StringFromCLSID(NCraftImageGenThumbnailGUID, &thumbExtGUID);
    lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(thumbExtGUID);

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_CREATE_SUB_KEY, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    lpSubKey = L"InProcServer32";
    res = RegCreateKeyEx(hkey, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)dllName, (DWORD)(std::wstring(dllName).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    lpKeyValue = L"Apartment";

    res = RegSetValueEx(hkey, L"ThreadingModel", 0, REG_SZ, (BYTE*)lpKeyValue.c_str(), (DWORD)((lpKeyValue.size() + 1U) * 2U));
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.glb\\shellex\\{E357FCCD-A995-4576-B01F-234630154E96}";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    std::wstring thumbExtGUIDStr(thumbExtGUID);

    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)thumbExtGUIDStr.c_str(), (DWORD)(thumbExtGUIDStr.size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\glb_auto_file\\shellex\\{E357FCCD-A995-4576-B01F-234630154E96}";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }


    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)thumbExtGUIDStr.c_str(), (DWORD)(thumbExtGUIDStr.size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);


    //**************************************************************************************************************
    // Register for file types

    lpSubKey = L"Software\\Classes\\.las\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.laz\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.pcd\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);


    lpSubKey = L"Software\\Classes\\.gltf\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\.glb\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    lpSubKey = L"Software\\Classes\\gltf_auto_file\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

   
    lpSubKey = L"Software\\Classes\\glb_auto_file\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);


    lpSubKey = L"Software\\Classes\\Directory\\ShellEx\\ContextMenuHandlers\\NCraftImageGen";

    res = RegCreateKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, NULL, REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkey, &lpDisp);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }
    res = RegSetValueEx(hkey, NULL, 0, REG_SZ, (BYTE*)std::wstring(menuExtGUID).c_str(), (DWORD)(std::wstring(menuExtGUID).size() + 1U) * 2U);
    if (res != ERROR_SUCCESS)
    {
        return E_UNEXPECTED;
    }

    RegCloseKey(hkey);

    //
    //**************************************************************************************************************


    // Put extension on the approved list
    lpSubKey = L"SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Shell Extensions\\Approved";

    if (ERROR_SUCCESS == RegOpenKeyEx(HKEY_CURRENT_USER, lpSubKey.c_str(), 0, KEY_ALL_ACCESS, &hkey))
    {
        res = RegSetValueEx(hkey, std::wstring(menuExtGUID).c_str(), 0, REG_SZ, (BYTE*)dllName, (DWORD)(std::wstring(dllName).size() + 1U) * 2U);
        if (res != ERROR_SUCCESS)
        {
            return E_UNEXPECTED;
        }

        res = RegSetValueEx(hkey, std::wstring(thumbExtGUID).c_str(), 0, REG_SZ, (BYTE*)dllName, (DWORD)(std::wstring(dllName).size() + 1U) * 2U);
        if (res != ERROR_SUCCESS)
        {
            return E_UNEXPECTED;
        }
        RegCloseKey(hkey);
    }


    SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, NULL, NULL);

    return S_OK;
}

HRESULT DllUnregisterServer()
{
    HKEY hkey;
    wchar_t* tempStr = nullptr;
    DWORD res = StringFromCLSID(NCraftImageGenMenuGUID, &tempStr);

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

    res = StringFromCLSID(NCraftImageGenThumbnailGUID, &tempStr);

    lpSubKey = L"Software\\Classes\\CLSID\\" + std::wstring(tempStr) + L"\\InprocServer32";

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



    SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, NULL, NULL);

    return S_OK;
}
#endif
