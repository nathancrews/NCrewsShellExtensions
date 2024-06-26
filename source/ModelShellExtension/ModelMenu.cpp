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


#include "priv.h"
#include "ModelShellExtension.h"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <process.h>
#include "Renderers/RenderGLTFToImage.h"
#include "NCraftImageGen.h"
#include "ModelMenu.h"


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

    try
    {
        open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

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

        utility::LogInfo("Model: Initialize Context Menu...");

        m_filePaths.clear();

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
                        m_filePaths.push_back(testPath);
                    }
                    else
                    {
                        for (std::string pcext : NCrewsImageGen::ModelFileExtensions)
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
    }
    catch (...)
    {
        utility::LogInfo("Model: Initialize Context Menu....catch!");
        hr = E_FAIL;
    }

    return hr;
}

// IContextMenu
HRESULT ModelMenu::QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags)
{
    try
    {
        open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

        if ((m_filePaths.size() == 0) || (uFlags & CMF_DEFAULTONLY))
        {
            utility::LogInfo("QueryContextMenu called....exiting no selected files or CMF_DEFAULTONLY");
            return MAKE_HRESULT(SEVERITY_SUCCESS, FACILITY_NULL, 0);
        }

        if (((uFlags & 0x000F) == CMF_NORMAL) || (uFlags & CMF_EXPLORE))
        {
            utility::LogInfo("QueryContextMenu called....");

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
    }
    catch (...)
    {
        utility::LogInfo("Model: QueryContextMenu....catch!");
    }

    return MAKE_HRESULT(SEVERITY_SUCCESS, 0, (USHORT)(0));
}

HRESULT ModelMenu::InvokeCommand(LPCMINVOKECOMMANDINFO lpici)
{
    HRESULT hr = E_FAIL;

    try
    {
        open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

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

        utility::LogInfo("Model: Menu Invoke Command called....");

        std::vector<std::filesystem::path> filesToImage;

        for (std::wstring fPath : m_filePaths)
        {
            std::filesystem::path cmdPath = fPath;
            filesToImage.push_back(cmdPath);
        }

        tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> renderResults;

        hr = S_OK;
        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());

        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings");
        }

        //std::string myCmd = "D:\\Projects\\LandXML2glTF\\LandXML2glTF\\build2\\bin\\Release\\LXML2GLTF.exe D:\\Projects\\LandXML2glTF\\LandXML2glTF\\LandXML\\subdivision-2.0\\subdivision-2.0.xml";
        //std::system(myCmd.c_str());
        //filesToImage.push_back(std::filesystem::path("D:\\Projects\\LandXML2glTF\\LandXML2glTF\\LandXML\\subdivision-2.0\\subdivision-2.0.gltf"));

        NCrewsImageGen::RenderModelsToImages(g_AppPath, filesToImage, m_imageGenSettings, renderResults);

        m_filePaths.clear();

        utility::LogInfo("Model: Menu Invoke Command called....Finished");
    }
    catch (...)
    {
        utility::LogInfo("Model: Menu Invoke Command....catch!");
    }

    return hr;
}

