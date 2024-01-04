#include "priv.h"
#include "ModelShellExtension.h"
#include "ModelMenu.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "wintoastlib.h"
#include "Renderers/RenderGLTFToImage.h"

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults);

class ModelCustomHandler : public WinToastLib::IWinToastHandler
{
public:

    ModelCustomHandler()
    {
        DllAddRef();
        utility::LogInfo("IWinToastHandler: CustomHandler() called");
    }
    ~ModelCustomHandler()
    {
        DllRelease();
        utility::LogInfo("IWinToastHandler: ~CustomHandler() called");
    }

    void toastActivated() const
    {
        utility::LogInfo("IWinToastHandler: Activated");
       // exit(0);
    }

    void toastActivated(int actionIndex) const
    {
        utility::LogInfo("IWinToastHandler: The user clicked on action");

       // exit(16 + actionIndex);
    }

    void toastDismissed(WinToastDismissalReason state) const
    {
        switch (state)
        {
            case UserCanceled:
                utility::LogInfo("IWinToastHandler: UserCanceled");

              //  exit(1);
                break;
            case TimedOut:
                utility::LogInfo("IWinToastHandler: TimedOut");

              //  exit(2);
                break;
            case ApplicationHidden:
                utility::LogInfo("IWinToastHandler: ApplicationHidden");

               // exit(3);
                break;
            default:
                utility::LogInfo("IWinToastHandler: not activated");

               // exit(4);
                break;
        }


    }

    void toastFailed() const
    {
        utility::LogInfo("IWinToastHandler: toastFailed");
    //    exit(5);
    }
};


ModelMenu::ModelMenu() : m_ObjRefCount(1)
{
    DllAddRef();
}

ModelMenu::~ModelMenu()
{
    DllRelease();
}

// IShellExtInit
HRESULT ModelMenu::Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID)
{
    HRESULT hr = E_FAIL;

    if (!pdtobj || g_DllModuleRefCount == 0)
    {
        return hr;
    }

    IShellItemArray* items = nullptr;
    hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

    if (!SUCCEEDED(hr) || !items)
    {
        return hr;
    }

    m_filePaths.clear();
    open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);
    utility::LogInfo("Model: Initialize Context Menu...");

    DWORD fcount = 0;
    items->GetCount(&fcount);

    for (DWORD i = 0; i < fcount; i++)
    {
        IShellItem* pRet = nullptr;
        LPWSTR nameBuffer = nullptr;

        items->GetItemAt(i, &pRet);
        if (pRet)
        {
            pRet->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &nameBuffer);
        }

        if (nameBuffer && wcslen(nameBuffer) > 0)
        {
            LPWSTR nameBufferCopy = new WCHAR[wcslen(nameBuffer) + 2];
            if (nameBufferCopy)
            {
                wcscpy(nameBufferCopy, nameBuffer);
                CoTaskMemFree(nameBuffer);

                std::filesystem::path testPath = nameBufferCopy;

                if (std::filesystem::is_directory(testPath))
                {
                    NCraftImageGen::GetFileNamesFromDirectory(testPath, NCraftImageGen::ModelFileExtensions, m_filePaths);
                }
                else
                {
                    for (std::string pcext : NCraftImageGen::ModelFileExtensions)
                    {
                        if (!testPath.extension().compare(pcext))
                        {
                            m_filePaths.push_back(testPath);
                            break;
                        }
                    }
                }

                delete[]nameBufferCopy;
            }
        }

        pRet->Release();
    }

    items->Release();

    if (m_filePaths.size() == 0)
    {
        hr = E_FAIL;
    }

    utility::LogInfo("Model: Initialize Context Menu...finished");

    return hr;
}

// IContextMenu
HRESULT ModelMenu::QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags)
{
    utility::LogInfo("QueryContextMenu called....");

    if ((m_filePaths.size() == 0) || (uFlags & CMF_DEFAULTONLY))
    {
        utility::LogInfo("QueryContextMenu called....exiting no selected files or CMF_DEFAULTONLY");
        return MAKE_HRESULT(SEVERITY_SUCCESS, FACILITY_NULL, 0);
    }

    if (((uFlags & 0x000F) == CMF_NORMAL) || (uFlags & CMF_EXPLORE))
    {
        m_idCmdFirst = idCmdFirst;

        std::wstring menuItemName = L"Generate glTF/GLB Image";

        if (m_filePaths.size() > 1)
        {
            WCHAR fileCountStr[MAX_PATH] = { 0 };
            _swprintf(fileCountStr, L"%zd", m_filePaths.size());

            menuItemName = L"Generate " + std::wstring(fileCountStr) + L" glTF/GLB Images";
        }

        LPWSTR menuItemNameStr = nullptr;

        menuItemNameStr = (LPWSTR)CoTaskMemAlloc((menuItemName.size() + 1) * sizeof(WCHAR));

        if (!menuItemNameStr)
        {
            MAKE_HRESULT(SEVERITY_ERROR, 0, (USHORT)(0));
        }


        //MENUITEMINFO menuInfoSep = {};
        //menuInfoSep.cbSize = sizeof(MENUITEMINFO);
        //menuInfoSep.fMask = MIIM_FTYPE;
        //menuInfoSep.fType = MFT_SEPARATOR;

        //InsertMenuItem(hmenu, 0, TRUE, &menuInfoSep);

        wcscpy(menuItemNameStr, menuItemName.c_str());

        MENUITEMINFO menuInfo = {};
        menuInfo.cbSize = sizeof(MENUITEMINFO);
        menuInfo.fMask = MIIM_STRING | MIIM_ID;
        menuInfo.dwTypeData = menuItemNameStr;
        menuInfo.wID = idCmdFirst;

        if (!InsertMenuItem(hmenu, 0, TRUE, &menuInfo))
        {
            utility::LogInfo("Model: QueryContextMenu....ERROR");
            return HRESULT_FROM_WIN32(GetLastError());
        }

        CoTaskMemFree(menuItemNameStr);

        utility::LogInfo("Model: QueryContextMenu....Added Menu item");

        return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(1));
    }

    utility::LogInfo("QueryContextMenu....No menu added");

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(0));
}

HRESULT ModelMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

    utility::LogInfo("Model: Menu Invoke Command called....");

    HRESULT hr = E_FAIL;

    if (!lpici)
    {
        hr = E_INVALIDARG;

        utility::LogInfo("Model: Menu Invoke passed bad data");
        return hr;
    }

    UINT const idCmd = LOWORD(lpici->lpVerb);

    if (m_filePaths.size() == 0)
    {
        utility::LogInfo("Model: m_filePaths is ZERO\n");
        return hr;
    }

    if (idCmd > 50)
    {
        utility::LogInfo("Model: Menu Command ID is not ZERO: {}, m_idCmdFirst: {}\n", idCmd, m_idCmdFirst);
        return hr;
    }

    std::vector<std::filesystem::path> filesToImage;

    for (std::wstring fPath : m_filePaths)
    {
        std::filesystem::path cmdPath = fPath;
        filesToImage.push_back(cmdPath);
    }

    tbb::concurrent_vector<NCraftImageGen::ImageGenResult> renderResults;

    hr = S_OK;
    try
    {
        NCraftImageGen::RenderModelsToImages(g_AppPath, filesToImage, renderResults);
        SendNotificationMessages(renderResults);

        m_filePaths.clear();
    }
    catch (/*CMemoryException* e*/...)
    {
        utility::LogInfo("Model: Error: RenderToImage crashed...\n");
        hr = E_FAIL;
    }

    utility::LogInfo("Model: Menu Invoke Command called....Finished");

    return hr;
}

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults)
{
    if (!WinToastLib::WinToast::isCompatible())
    {
        utility::LogInfo("Model: WinToast Error, your system is not supported!");
    }

    std::wstring infoText;
    WCHAR pointCountStr[MAX_PATH] = { 0 };
    WCHAR timeStr[MAX_PATH] = { 0 };
    WCHAR fileSizeStr[MAX_PATH] = { 0 };
    UINT millionVal = 1000000;
    UINT kVal = 1000;

    if (imageResults.size() > 5)
    {
        WinToastLib::WinToastTemplate templ(WinToastLib::WinToastTemplate::ImageAndText02);

        templ.setTextField(imageResults[0].m_ImageName.filename(), WinToastLib::WinToastTemplate::FirstLine);

        if (std::filesystem::exists(imageResults[0].m_ImageName))
        {
            templ.setImagePath(imageResults[0].m_ImageName);
        }

        _swprintf(fileSizeStr, L"Generated: %d images", (int)imageResults.size());
        templ.setTextField(fileSizeStr, WinToastLib::WinToastTemplate::SecondLine);

        templ.setDuration(WinToastLib::WinToastTemplate::Short);

        templ.setExpiration(10000);

        if (WinToastLib::WinToast::instance()->showToast(templ, new ModelCustomHandler()) < 0)
        {
            utility::LogInfo("Model: WinToast Error, could not launch toast notification!");
        }
    }
    else
    {
        for (NCraftImageGen::ImageGenResult& result : imageResults)
        {
            WinToastLib::WinToastTemplate templ(WinToastLib::WinToastTemplate::ImageAndText04);

            templ.setDuration(WinToastLib::WinToastTemplate::Short);

            templ.setTextField(result.m_ImageName.filename(), WinToastLib::WinToastTemplate::FirstLine);
            templ.setTextField(result.m_FileName.filename(), WinToastLib::WinToastTemplate::SecondLine);

            if (std::filesystem::exists(result.m_ImageName))
            {
                templ.setImagePath(result.m_ImageName);
            }

            if (result.m_fileSize > 1048576 * 1000)
            {
                _swprintf(fileSizeStr, L"File size: %0.2f GB", (double)(result.m_fileSize) / (double)(1048576 * 1000));
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
                _swprintf(timeStr, L"%0.2fms", result.m_processTimeSeconds * 1000);
            }

            templ.setTextField(infoText, WinToastLib::WinToastTemplate::ThirdLine);

            templ.setAttributionText(fileSizeStr);
            templ.setExpiration(10000);

            if (WinToastLib::WinToast::instance()->showToast(templ, new ModelCustomHandler()) < 0)
            {
                utility::LogInfo("Model: WinToast Error, could not launch toast notification!");
            }
        }
    }
}