#include "priv.h"
#include "NCraftImageGen.h"
#include "NCraftImageGenMenu.h"
#include <strsafe.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "Renderers/RenderToImage.h"

extern long g_DllModuleRefCount;
extern TCHAR g_DllModelName[MAX_PATH];
extern std::filesystem::path G_AppPath;

void print_fcn(const std::string& logString)
{
    std::filesystem::path logFilePath = G_AppPath;

    logFilePath.replace_filename("1_imageGen");
    logFilePath.replace_extension("log");

    std::fstream fs;
    fs.open(logFilePath, std::fstream::out | std::fstream::app);

    fs << logString;
    fs << "\n";

    fs.flush();
    fs.close();
}

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
    std::filesystem::path appPath = g_DllModelName;
    appPath = appPath.remove_filename();

    G_AppPath = appPath;
    open3d::utility::Logger::GetInstance().SetPrintFunction(print_fcn);


    IShellItemArray* items;
    HRESULT hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

    DWORD fcount = 0;
    items->GetCount(&fcount);

    //HRESULT hr = pdtobj->QueryInterface(&_pdtobj);
    //// Get the path to the drop target folder
    if (SUCCEEDED(hr) && fcount > 0)
    {
        for (DWORD i = 0; i < fcount; i++)
        {
            IShellItem* pRet;
            LPWSTR nameBuffer;
            items->GetItemAt(i, &pRet);
            pRet->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &nameBuffer);
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
        HRESULT hr = E_FAIL;
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

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(1));  // indicate that we added 2 verbs.
}

HRESULT NCraftImageGenContextMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    HRESULT hr = E_FAIL;

    UINT const idCmd = LOWORD(lpici->lpVerb);

//    utility::LogInfo("Menu Command ID registered: {}, passed {}\n", m_idCmdFirst, idCmd);

    if (idCmd != 0)
    {
        return hr;
    }

    if (m_filePaths.size() == 0)
    {
        return hr;
    }

    if (std::filesystem::is_directory(m_filePaths[0]) || m_filePaths.size() > 5)
    {
        int retval = MessageBox(nullptr, m_filePaths[0].c_str(), L"    Proceed to Generate Images?", MB_YESNO);

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
            NCraftImageGen::RenderToImages(G_AppPath, filesToImage);
        }
        catch (/*CMemoryException* e*/...)
        {
            hr = E_FAIL;
        }

    }

    return hr;
}

