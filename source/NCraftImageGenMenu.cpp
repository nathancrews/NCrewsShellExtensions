#include "priv.h"
#include "NCraftImageGen.h"
#include "NCraftImageGenMenu.h"
#include <strsafe.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "wintoastlib.h"
#include "Renderers/RenderToImage.h"

class CustomHandler : public WinToastLib::IWinToastHandler
{
public:
    void toastActivated() const
    {
        std::wcout << L"The user clicked in this toast" << std::endl;
       // exit(0);
    }

    void toastActivated(int actionIndex) const
    {
        std::wcout << L"The user clicked on action #" << actionIndex << std::endl;
       // exit(16 + actionIndex);
    }

    void toastDismissed(WinToastDismissalReason state) const
    {
        switch (state)
        {
            case UserCanceled:
                std::wcout << L"The user dismissed this toast" << std::endl;
              //  exit(1);
                break;
            case TimedOut:
                std::wcout << L"The toast has timed out" << std::endl;
              //  exit(2);
                break;
            case ApplicationHidden:
                std::wcout << L"The application hid the toast using ToastNotifier.hide()" << std::endl;
               // exit(3);
                break;
            default:
                std::wcout << L"Toast not activated" << std::endl;
               // exit(4);
                break;
        }
    }

    void toastFailed() const
    {
        std::wcout << L"Error showing current toast" << std::endl;
    //    exit(5);
    }
};


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
    HRESULT hr = E_FAIL;

    if (!pdtobj)
    {
        return E_FAIL;
    }

    IShellItemArray* items = nullptr;
    hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

    if (!SUCCEEDED(hr))
    {
        return E_FAIL;
    }

    utility::LogInfo("Initializing Context Menu...");

    DWORD fcount = 0;
    items->GetCount(&fcount);

    hr = pdtobj->QueryInterface(&_pdtobj);
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
    if (!WinToastLib::WinToast::isCompatible())
    {
        utility::LogInfo("WinToast Error, your system is not supported!");
    }

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

    std::vector<std::filesystem::path> filesToImage;

    for (std::wstring fPath : m_filePaths)
    {
        std::filesystem::path cmdPath = fPath;
        filesToImage.push_back(cmdPath);
    }

    tbb::concurrent_vector<NCraftImageGen::ImageGenResult> renderResults;

    if (filesToImage.size() > 0)
    {
        hr = S_OK;
        try
        {
            NCraftImageGen::RenderToImages(g_AppPath, filesToImage, renderResults);
        }
        catch (/*CMemoryException* e*/...)
        {
            utility::LogInfo("Error: RenderToImage crashed...\n");
            hr = E_FAIL;
        }

    }

    for (NCraftImageGen::ImageGenResult& result : renderResults)
    {
        WinToastLib::WinToastTemplate templ(WinToastLib::WinToastTemplate::ImageAndText04);

        templ.setDuration(WinToastLib::WinToastTemplate::Short);

        templ.setTextField(result.m_ImageName.filename(), WinToastLib::WinToastTemplate::FirstLine);
        templ.setTextField(result.m_FileName.filename(), WinToastLib::WinToastTemplate::SecondLine);

        if (std::filesystem::exists(result.m_ImageName))
        {
            templ.setImagePath(result.m_ImageName);
        }

        std::wstring infoText;
        WCHAR pointCountStr[100] = { 0 };
        WCHAR timeStr[100] = { 0 };
        WCHAR fileSizeStr[100] = { 0 };


        if (result.m_fileSize > 1048576*1000)
        {
            _swprintf(fileSizeStr, L"File size: %0.2f GB", (double)(result.m_fileSize) / (double)(1048576*1000));
        }
        else if (result.m_fileSize > 1048576)
        {
            _swprintf(fileSizeStr, L"File size: %0.2f MB", (double)(result.m_fileSize) / (double)(1048576));
        }
        else
        {
            _swprintf(fileSizeStr, L"File size: %0.2f KB", (double)result.m_fileSize / (double)1048);
        }

        if (result.m_processTimeSeconds > 1.0)
        {
            _swprintf(timeStr, L"%0.2fs", result.m_processTimeSeconds);
        }
        else
        {
            _swprintf(timeStr, L"%0.2fms", result.m_processTimeSeconds*1000);
        }

        UINT millionVal = 1000000;
        UINT kVal = 1000;

        if (result.m_modelType == 0)
        {
            if (result.m_pointCount > millionVal)
            {
                _swprintf(pointCountStr, L"%0.2f M", (double)(result.m_pointCount) / (double)millionVal);
            }
            else if (result.m_pointCount > kVal)
            {
                _swprintf(pointCountStr, L"%0.2f K", (double)(result.m_pointCount) / (double)kVal);
            }
            else
            {
                _swprintf(pointCountStr, L"%d", result.m_pointCount);
            }

            infoText = L"Points: " + std::wstring(pointCountStr) + L", Time: " + timeStr;
        }
        else
        {
            infoText = L"Time: " + std::wstring(timeStr);
        }

        templ.setTextField(infoText, WinToastLib::WinToastTemplate::ThirdLine);

        templ.setAttributionText(fileSizeStr);

        if (WinToastLib::WinToast::instance()->showToast(templ, new CustomHandler()) < 0)
        {
            utility::LogInfo("WinToast Error, could not launch your toast notification!");
        }
    }
 
    return hr;
}
