#include "priv.h"
#include "CloudShellExtension.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "wintoastlib.h"
#include "Renderers/RenderPointCloudToImage.h"
#include "NCraftImageGen.h"
#include "CloudMenu.h"

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults);

class CloudCustomHandler : public WinToastLib::IWinToastHandler
{
public:

    CloudCustomHandler()
    {
        DllAddRef();
        utility::LogInfo("Cloud IWinToastHandler: CustomHandler() called");
    }
    ~CloudCustomHandler()
    {
        utility::LogInfo("Cloud IWinToastHandler: ~CustomHandler() called");
        DllRelease();
    }

    void toastActivated() const
    {
//        utility::LogInfo("IWinToastHandler: Activated");
        //std::wcout << L"The user clicked in this toast" << std::endl;
       // exit(0);
    }

    void toastActivated(int actionIndex) const
    {
//        utility::LogInfo("IWinToastHandler: The user clicked on action");
        //std::wcout << L"The user clicked on action #" << actionIndex << std::endl;
       // exit(16 + actionIndex);
    }

    void toastDismissed(WinToastDismissalReason state) const
    {
        switch (state)
        {
            case UserCanceled:
//                utility::LogInfo("IWinToastHandler: UserCanceled");

              //  exit(1);
                break;
            case TimedOut:
//                utility::LogInfo("IWinToastHandler: TimedOut");

              //  exit(2);
                break;
            case ApplicationHidden:
//                utility::LogInfo("IWinToastHandler: ApplicationHidden");

               // exit(3);
                break;
            default:
//                utility::LogInfo("IWinToastHandler: not activated");

               // exit(4);
                break;
        }
    }

    void toastFailed() const
    {
    //    exit(5);
    }
};


CloudMenu::CloudMenu() : m_ObjRefCount(1)
{
    DllAddRef();
}

CloudMenu::~CloudMenu()
{
    DllRelease();
}

// IShellExtInit
HRESULT CloudMenu::Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID)
{
    HRESULT hr = E_FAIL;

    try
    {

        if (!pdtobj)
        {
            return hr;
        }

        IShellItemArray* items = nullptr;
        hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

        if (!SUCCEEDED(hr) || !items)
        {
            return hr;
        }

        utility::LogInfo("Cloud: Initialize Context Menu...");

        m_filePaths.clear();
        
        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());

        if (!NCraftImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings");
        }

        DWORD fcount = 0;
        items->GetCount(&fcount);

        for (DWORD i = 0; i < fcount; i++)
        {
            IShellItem* pRet = nullptr;
            LPWSTR nameBuffer = nullptr;

            items->GetItemAt(i, &pRet);
            if (pRet)
            {
                pRet->GetDisplayName(SIGDN_FILESYSPATH, &nameBuffer);
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
                        m_filePaths.push_back(testPath);
                    }
                    else
                    {
                        for (std::string pcext : NCraftImageGen::PointcloudFileExtensions)
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

        utility::LogInfo("Cloud: Initialize Context Menu...finished");
    }
    catch (...)
    {
        utility::LogInfo("Cloud: Initialize Context Menu....catch!");
    }

    return hr;
}

// IContextMenu
HRESULT CloudMenu::QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags)
{
    HRESULT hres = MAKE_HRESULT(SEVERITY_SUCCESS, FACILITY_NULL, 0);

    try
    {
        if ((m_filePaths.size() == 0) || (uFlags & CMF_DEFAULTONLY))
        {
            utility::LogInfo("Cloud: QueryContextMenu called....exiting no selected files or CMF_DEFAULTONLY");
            return hres;
        }

        if (((uFlags & 0x000F) == CMF_NORMAL) || (uFlags & CMF_EXPLORE))
        {
            utility::LogInfo("Cloud: QueryContextMenu called....");

            m_idCmdFirst = idCmdFirst;

            std::wstring menuItemName = L"Generate Pointcloud Image";
            std::wstring menuMergeItemName = L"Generate Merged Pointcloud Image";

            if (m_filePaths.size() > 1)
            {
                WCHAR fileCountStr[MAX_PATH] = { 0 };
                _swprintf(fileCountStr, L"%zd", m_filePaths.size());

                menuItemName = L"Generate " + std::wstring(fileCountStr) + L" Pointcloud Images";
            }

            LPWSTR menuItemNameStr = nullptr;
            LPWSTR menuMergeItemNameStr = nullptr;

            menuItemNameStr = (LPWSTR)CoTaskMemAlloc((menuItemName.size() + 1) * sizeof(WCHAR));
            menuMergeItemNameStr = (LPWSTR)CoTaskMemAlloc((menuMergeItemName.size() + 1) * sizeof(WCHAR));

            if (!menuItemNameStr || !menuMergeItemNameStr)
            {
                hres = MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NULL, 0);
                return hres; 
            }

            wcscpy(menuItemNameStr, menuItemName.c_str());
            wcscpy(menuMergeItemNameStr, menuMergeItemName.c_str());

            MENUITEMINFO menuInfo = {};
            menuInfo.cbSize = sizeof(MENUITEMINFO);
            menuInfo.fMask = MIIM_STRING | MIIM_ID;
            menuInfo.dwTypeData = menuItemNameStr;
            menuInfo.wID = idCmdFirst;

            if (!InsertMenuItem(hmenu, 0, TRUE, &menuInfo))
            {
                utility::LogInfo("QueryContextMenu....ERROR 1");
                return HRESULT_FROM_WIN32(GetLastError());
            }

            if (m_filePaths.size() > 1)
            {
                MENUITEMINFO menuInfo2 = {};
                menuInfo2.cbSize = sizeof(MENUITEMINFO);
                menuInfo2.fMask = MIIM_STRING | MIIM_ID;
                menuInfo2.dwTypeData = menuMergeItemNameStr;
                menuInfo2.wID = idCmdFirst + 1;

                if (!InsertMenuItem(hmenu, 1, TRUE, &menuInfo2))
                {
                    utility::LogInfo("QueryContextMenu....ERROR 2");
                    return HRESULT_FROM_WIN32(GetLastError());
                }
            }

            CoTaskMemFree(menuItemNameStr);
            CoTaskMemFree(menuMergeItemNameStr);

            utility::LogInfo("Cloud: QueryContextMenu....Added Menu item");

            hres = MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(1));

            if (m_filePaths.size() > 1)
            {
                hres = MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(2));
            }

            return hres;
        }

        utility::LogInfo("Cloud: QueryContextMenu....No menu added");
    }
    catch (...)
    {
        utility::LogInfo("Cloud: QueryContextMenu....catch!");
    }

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(0));
}

HRESULT CloudMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    HRESULT hr = E_FAIL;

    try
    {
        if (!lpici)
        {
            hr = E_INVALIDARG;

            utility::LogInfo("Cloud: Menu Invoke passed bad data");
            return hr;
        }

        utility::LogInfo("Cloud: Menu Invoke Command called....");

        UINT const idCmd = LOWORD(lpici->lpVerb);

        if (m_filePaths.size() == 0)
        {
            utility::LogInfo("Cloud: m_filePaths is ZERO\n");
            return hr;
        }

        if (idCmd > 50)
        {
            utility::LogInfo("Cloud: Menu Command ID is not ZERO: {}, m_idCmdFirst: {}\n", idCmd, m_idCmdFirst);
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

            if (idCmd == 0)
            {
                utility::LogInfo("Cloud: Menu command 0 called...");
                NCraftImageGen::RenderPointcloudFiles(g_AppPath, filesToImage, m_imageGenSettings, renderResults);
            }
            else if (idCmd == 1)
            {
                utility::LogInfo("Cloud: Menu command 1 called...");

                NCraftImageGen::RenderPointcloudFilesToSingleImage(g_AppPath, filesToImage, m_imageGenSettings, renderResults);
            }

            SendNotificationMessages(renderResults);

            m_filePaths.clear();
        }
        catch (...)
        {
            utility::LogInfo("Cloud: Error: RenderPointcloudFiles crashed...");
            hr = E_FAIL;
        }

        utility::LogInfo("Cloud: Menu Invoke Command called....Finished");
    }
    catch (...)
    {
        utility::LogInfo("Cloud: Menu Invoke Command....catch!");
    }

    return hr;
}

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults)
{
    try
    {
        if (!WinToastLib::WinToast::isCompatible())
        {
            utility::LogInfo("Cloud: WinToast Error, system is not supported!");
        }

        std::wstring infoText;
        WCHAR pointCountStr[MAX_PATH] = { 0 };
        WCHAR timeStr[MAX_PATH] = { 0 };
        WCHAR fileSizeStr[MAX_PATH] = { 0 };
        uintmax_t totalPointsProcessed = 0;
        double totalProcessingTime = 0;
        double totalFileSize = 0.0;
        UINT millionVal = 1000000;
        UINT kVal = 1000;

        WinToastLib::WinToast::instance()->setAppName(g_AppName);
        WinToastLib::WinToast::instance()->setAppUserModelId(g_AppName);


        if (imageResults.size() > 4)
        {
            for (NCraftImageGen::ImageGenResult& result : imageResults)
            {
                totalPointsProcessed += result.m_pointCount;
                totalProcessingTime += result.m_processTimeSeconds;
                totalFileSize += result.m_fileSize;
            }

            WinToastLib::WinToastTemplate templ(WinToastLib::WinToastTemplate::ImageAndText04);

            templ.setDuration(WinToastLib::WinToastTemplate::Short);
            templ.setExpiration(50000);

            if (std::filesystem::exists(imageResults[0].m_ImageName))
            {
                templ.setImagePath(imageResults[0].m_ImageName);
            }

            _swprintf(fileSizeStr, L"Generated: %d images", (int)imageResults.size());

            templ.setTextField(fileSizeStr, WinToastLib::WinToastTemplate::FirstLine);

            if (totalFileSize > 1048576 * 1000)
            {
                _swprintf(fileSizeStr, L"Total File size: %0.2f GB", (double)(totalFileSize) / (double)(1048576 * 1000));
            }
            else if (totalFileSize > 1048576)
            {
                _swprintf(fileSizeStr, L"Total File size: %0.2f MB", (double)(totalFileSize) / (double)(1048576));
            }
            else
            {
                _swprintf(fileSizeStr, L"Total File size: %0.2f KB", (double)(totalFileSize / (double)1048));
            }

            templ.setTextField(fileSizeStr, WinToastLib::WinToastTemplate::SecondLine);

            if (WinToastLib::WinToast::instance()->showToast(templ, new CloudCustomHandler()) < 0)
            {
                utility::LogInfo("Cloud: WinToast Error, could not launch toast notification!");
            }
        }
        else
        {
            for (NCraftImageGen::ImageGenResult& result : imageResults)
            {
                WinToastLib::WinToastTemplate templ(WinToastLib::WinToastTemplate::ImageAndText04);

                templ.setDuration(WinToastLib::WinToastTemplate::Short);
                templ.setExpiration(50000);

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

                templ.setTextField(fileSizeStr, WinToastLib::WinToastTemplate::ThirdLine);

                if (WinToastLib::WinToast::instance()->showToast(templ, new CloudCustomHandler()) < 0)
                {
                    utility::LogInfo("Cloud: WinToast Error, could not launch toast notification!");
                }

            }
        }
    }
    catch (...)
    {
        utility::LogInfo("Cloud: send notification....catch!");
    }
}