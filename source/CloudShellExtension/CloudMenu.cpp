////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan Crews, NCrews Software
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
#include "CloudShellExtension.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "Renderers/RenderPointCloudToImage.h"
#include "NCraftImageGen.h"
#include "CloudMenu.h"


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
        open3d::utility::Logger::GetInstance().SetPrintFunction(cloud_print_fcn);

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

        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
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
                        for (std::string pcext : NCrewsImageGen::PointcloudFileExtensions)
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
        open3d::utility::Logger::GetInstance().SetPrintFunction(cloud_print_fcn);

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
        open3d::utility::Logger::GetInstance().SetPrintFunction(cloud_print_fcn);

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

        tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> renderResults;

        hr = S_OK;
        try
        {

            if (idCmd == 0)
            {
                utility::LogInfo("Cloud: Menu command 0 called...");
                NCrewsImageGen::RenderPointcloudFiles(g_AppPath, filesToImage, m_imageGenSettings, renderResults);
            }
            else if (idCmd == 1)
            {
                utility::LogInfo("Cloud: Menu command 1 called...");
                NCrewsImageGen::RenderPointcloudFilesToSingleImage(g_AppPath, filesToImage, m_imageGenSettings, renderResults);
            }

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

