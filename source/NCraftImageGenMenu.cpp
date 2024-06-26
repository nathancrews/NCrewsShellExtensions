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

#include "priv.h"
#include "NCraftImageGen.h"
#include "NCraftImageGenMenu.h"
#include <strsafe.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "wintoastlib.h"
#include "Renderers/RenderPointcloudToImage.h"
#include "Renderers/RenderGLTFToImage.h"

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults);

class CustomHandler : public WinToastLib::IWinToastHandler
{
public:

    CustomHandler()
    {
        DllAddRef(); 
        utility::LogInfo("IWinToastHandler: CustomHandler() called");
    }
    ~CustomHandler()
    {
        DllRelease(); 
        utility::LogInfo("IWinToastHandler: ~CustomHandler() called");
    }

    void toastActivated() const
    {
        utility::LogInfo("IWinToastHandler: Activated");
        //std::wcout << L"The user clicked in this toast" << std::endl;
       // exit(0);
    }

    void toastActivated(int actionIndex) const
    {
        utility::LogInfo("IWinToastHandler: The user clicked on action");
        //std::wcout << L"The user clicked on action #" << actionIndex << std::endl;
       // exit(16 + actionIndex);
    }

    void toastDismissed(WinToastDismissalReason state) const
    {
        switch (state)
        {
            case UserCanceled:
                utility::LogInfo("IWinToastHandler: UserCanceled");
              //  std::wcout << L"The user dismissed this toast" << std::endl;
              //  exit(1);
                break;
            case TimedOut:
                utility::LogInfo("IWinToastHandler: TimedOut");

              //  std::wcout << L"The toast has timed out" << std::endl;
              //  exit(2);
                break;
            case ApplicationHidden:
                utility::LogInfo("IWinToastHandler: ApplicationHidden");
               // std::wcout << L"The application hid the toast using ToastNotifier.hide()" << std::endl;
               // exit(3);
                break;
            default:
                utility::LogInfo("IWinToastHandler: not activated");
               // std::wcout << L"Toast not activated" << std::endl;
               // exit(4);
                break;
        }
    }

    void toastFailed() const
    {
        utility::LogInfo("IWinToastHandler: toastFailed");
       // std::wcout << L"Error showing current toast" << std::endl;
    //    exit(5);
    }
};


NCraftImageGenContextMenu::NCraftImageGenContextMenu() : m_ObjRefCount(1)
{
    DllAddRef();
}

NCraftImageGenContextMenu::~NCraftImageGenContextMenu()
{
    DllRelease();
}

// IShellExtInit
HRESULT NCraftImageGenContextMenu::Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID)
{
    utility::LogInfo("Initialize Context Menu...");

    HRESULT hr = E_FAIL;

    if (!pdtobj)
    {
        return E_INVALIDARG;
    }

    m_filePaths.clear();

    IShellItemArray* items = nullptr;
    hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);

    if (!SUCCEEDED(hr) || !items)
    {
        return E_FAIL;
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
            pRet->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &nameBuffer);
        }

        if (nameBuffer && wcslen(nameBuffer) > 0)
        {
            LPWSTR nameBufferCopy = new WCHAR[wcslen(nameBuffer)+2];
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
                    for (std::string pcext : NCraftImageGen::ModelFileExtensions)
                    {
                        if (!testPath.extension().compare(pcext))
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

                delete []nameBufferCopy;
            }
        }

        pRet->Release();
    }

    items->Release();

    if (m_filePaths.size() == 0)
    {
        hr = E_FAIL;
    }

    utility::LogInfo("Initialize Context Menu...finished");

    return hr;
}

// IContextMenu
HRESULT NCraftImageGenContextMenu::QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags)
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
       
        std::wstring menuItemName = L"Generate Image Previews";

        if (m_filePaths.size() > 1)
        {
            WCHAR fileCountStr[MAX_PATH] = { 0 };
            _swprintf(fileCountStr, L"%zd", m_filePaths.size());

            menuItemName = L"Generate " + std::wstring(fileCountStr) + L" Image Previews";
        }
       
        LPWSTR menuItemNameStr = nullptr;

        menuItemNameStr = (LPWSTR)CoTaskMemAlloc((menuItemName.size() + 1) * sizeof(WCHAR));

        if (!menuItemNameStr)
        {
            MAKE_HRESULT(SEVERITY_ERROR, 0, (USHORT)(0));
        }

        wcscpy(menuItemNameStr, menuItemName.c_str());

        MENUITEMINFO menuInfo = {};
        menuInfo.cbSize = sizeof(MENUITEMINFO);
        menuInfo.fMask = MIIM_STRING | MIIM_ID;
        menuInfo.dwTypeData = menuItemNameStr;
        menuInfo.wID = idCmdFirst;

        if (!InsertMenuItem(hmenu, 0, TRUE, &menuInfo))
        {
            utility::LogInfo("QueryContextMenu....ERROR");
            return HRESULT_FROM_WIN32(GetLastError());
        }

        CoTaskMemFree(menuItemNameStr);

        utility::LogInfo("QueryContextMenu....Added Menu item");

        return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(1));
    }

    utility::LogInfo("QueryContextMenu....No menu added");

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(0));
}

HRESULT NCraftImageGenContextMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    utility::LogInfo("Menu Invoke Command called....");

    HRESULT hr = E_FAIL;

    if (!lpici)
    {
        hr = E_INVALIDARG;

        utility::LogInfo("Menu Invoke passed bad data");
        return hr;
    }

    UINT const idCmd = LOWORD(lpici->lpVerb);

    if (m_filePaths.size() == 0)
    {
        utility::LogInfo("m_filePaths is ZERO\n");
        return hr;
    }

    if (idCmd > 50)
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

    hr = S_OK;
    try
    {
        NCraftImageGen::RenderModelsToImages(g_AppPath, filesToImage, renderResults);
        NCraftImageGen::RenderPointcloudFiles(g_AppPath, filesToImage, renderResults);
        SendNotificationMessages(renderResults);

        m_filePaths.clear();
    }
    catch (/*CMemoryException* e*/...)
    {
        utility::LogInfo("Error: RenderToImage crashed...\n");
        hr = E_FAIL;
    }

    utility::LogInfo("Menu Invoke Command called....Finished");

    return hr;
}

void SendNotificationMessages(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& imageResults)
{
    if (!WinToastLib::WinToast::isCompatible())
    {
        utility::LogInfo("WinToast Error, your system is not supported!");
    }

    std::wstring infoText;
    WCHAR pointCountStr[MAX_PATH] = { 0 };
    WCHAR timeStr[MAX_PATH] = { 0 };
    WCHAR fileSizeStr[MAX_PATH] = { 0 };
    UINT millionVal = 1000000;
    UINT kVal = 1000;

    for (NCraftImageGen::ImageGenResult& result : imageResults)
    {
        if (result.m_pointCount > 0)
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
                utility::LogInfo("WinToast Error, could not launch toast notification!");
            }
        }
    }
}
