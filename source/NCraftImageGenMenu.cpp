#include "priv.h"
#include "NCraftImageGen.h"
#include "NCraftImageGenMenu.h"
#include <strsafe.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "Renderers/RenderToImage.h"


NCraftImageGenContextMenu::NCraftImageGenContextMenu() : m_ObjRefCount(1), _pdtobj(NULL)
{
    _szTargetFolder[0] = 0;
    DllAddRef();
}

NCraftImageGenContextMenu::~NCraftImageGenContextMenu()
{
    DllRelease();
}

// IShellExtInit
HRESULT NCraftImageGenContextMenu::Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID)
{
    IShellItemArray* items = nullptr;
    HRESULT hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

    if (hr != S_OK)
    {
        return E_FAIL;
    }

    DWORD fcount = 0;
    items->GetCount(&fcount);

    //HRESULT hr = pdtobj->QueryInterface(&_pdtobj);
    //// Get the path to the drop target folder
    if (SUCCEEDED(hr) && fcount > 0)
    {
        for (DWORD i = 0; i < fcount; i++)
        {
            IShellItem* pRet = nullptr;
            LPWSTR nameBuffer = nullptr;

            items->GetItemAt(i, &pRet);
            if (pRet)
            {
                pRet->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &nameBuffer);
            }

            if (nameBuffer)
            {
                std::filesystem::path testPath(std::move(nameBuffer));

                if (std::filesystem::is_directory(testPath))
                {
                    m_filePaths.push_back(testPath);
                }
                else
                {
                    for (std::string fext : NCraftImageGen::ModelFileExtensions)
                    {
                        if (!testPath.extension().compare(fext))
                        {
                            m_filePaths.push_back(testPath);
                            break;
                        }
                    }

                    for (std::string pcext : NCraftImageGen::PointcloudFileExtensions)
                    {
                        if (!testPath.extension().compare(pcext))
                        {
                            m_filePaths.push_back(testPath);
                            break;
                        }
                    }
                }
            }

            pRet->Release();
            CoTaskMemFree(nameBuffer);
        }

        items->Release();
    }

    if (m_filePaths.size() == 0)
    {
        hr = E_FAIL;
    }

    return hr;
}

// IContextMenu
HRESULT NCraftImageGenContextMenu::QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags)
{
    if (uFlags & CMF_DEFAULTONLY)
    {
        return MAKE_HRESULT(SEVERITY_SUCCESS, FACILITY_NULL, 0);
    }

    if (((uFlags & 0x000F) == CMF_NORMAL) || (uFlags & CMF_EXPLORE))
    {
        m_idCmdFirst = idCmdFirst;

        std::wstring menuItemName = L"Generate Image Preview";
        if (m_filePaths.size() > 1)
        {
            WCHAR fileCountStr[100] = { 0 };
            _swprintf(fileCountStr, L"%zd", m_filePaths.size());

            menuItemName = L"Generate " + std::wstring(fileCountStr) + L" Image Previews";
        }
        else if (m_filePaths.size() == 1)
        {
            if (std::filesystem::is_directory(m_filePaths[0]))
            {
                menuItemName = L"Generate Image Previews in Folder";
            }
        }

        MENUITEMINFO menuInfo = {};
        menuInfo.cbSize = sizeof(MENUITEMINFO);
        menuInfo.fMask = MIIM_STRING | MIIM_ID;
        menuInfo.dwTypeData = (LPWSTR)menuItemName.c_str();
        menuInfo.wID = idCmdFirst;

        if (!InsertMenuItem(hmenu, 0, TRUE, &menuInfo))
        {
            return HRESULT_FROM_WIN32(GetLastError());
        }

        return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(1));
    }

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(0));
}

HRESULT NCraftImageGenContextMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    HRESULT hr = E_FAIL;
    UINT const idCmd = LOWORD(lpici->lpVerb);

    if (m_filePaths.size() == 0)
    {
        utility::LogInfo("m_filePaths is ZERO\n");
        return hr;
    }

    if (idCmd != 0)
    {
        utility::LogInfo("Menu Command ID is not ZERO: {}, m_idCmdFirst: {}\n", idCmd, m_idCmdFirst);
        return hr;
    }

    if (std::filesystem::is_directory(m_filePaths[0]) || m_filePaths.size() > 5)
    {
        int retval = MessageBox(nullptr, L"    Proceed to Generate Images?", g_AppName.c_str(), MB_YESNO);

        if (retval != 6)
        {
            return hr;
        }
    }

    std::vector<std::filesystem::path> filesToImage;

    for (std::wstring fPath : m_filePaths)
    {
        std::filesystem::path cmdPath = fPath;
        filesToImage.push_back(cmdPath);
    }

    if (filesToImage.size() > 0)
    {
        hr = S_OK;
        try
        {
            NCraftImageGen::RenderToImages(g_AppPath, filesToImage);

            MessageBox(nullptr, L"    Image Generation Complete", g_AppName.c_str(), MB_OK);
        }
        catch (/*CMemoryException* e*/...)
        {
            utility::LogInfo("Error: RenderToImage crashed...\n");
            hr = E_FAIL;
        }

    }

    return hr;
}
