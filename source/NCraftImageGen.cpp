#include "NCraftImageGen.h"
#include "NCraftImageGenDll.h"
#include "NCraftImageGenMenuGUID.h"
#include "NCraftClassFactory.h"
#include "Renderers/RenderToImage.h"

// Standard DLL functions
BOOL DllMain(HINSTANCE hInstance, DWORD dwReason, void*)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        g_hinst = hInstance;
        DisableThreadLibraryCalls(hInstance);

        DWORD strSize = MAX_PATH;
        GetModuleFileNameW(hInstance, g_DllModelName, strSize);

        std::filesystem::path appPath = g_DllModelName;
        g_AppPath = appPath.remove_filename();

        open3d::utility::Logger::GetInstance().SetPrintFunction(print_fcn);
    }
    return TRUE;
}

HRESULT DllCanUnloadNow(void)
{
    // Only allow the DLL to be unloaded after all outstanding references have been released
    return (g_DllModuleRefCount == 0) ? S_OK : S_FALSE;

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

//    MessageBox(nullptr, L"DllGetClassObject", L"NCraftImageGen", MB_OK);

    if (!IsEqualCLSID(clsid, NCraftImageGenMenuGUID))
    {
        return CLASS_E_CLASSNOTAVAILABLE;
    }

    if (!ppv)
    {
        return E_INVALIDARG;
    }

    HRESULT res = E_UNEXPECTED;

    if (IsEqualCLSID(clsid, NCraftImageGenMenuGUID))
    {
        NCraftClassFactory* nCF = new NCraftClassFactory();
        if (nCF)
        {
            res = nCF->QueryInterface(riid, ppv);
 /*           if (res == S_OK)
            {
                MessageBox(nullptr, L"QueryInterface", L"NCraftImageGen", MB_OK);
            }*/

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



    SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST, NULL, NULL);

    return S_OK;
}