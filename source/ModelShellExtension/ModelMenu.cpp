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
#include <algorithm>
#include <cctype>
#include "Renderers/RenderGLTFToImage.h"
#include "NCraftImageGen.h"
#include "ModelMenu.h"
#include "ModelMenuGUID.h"

namespace
{
const wchar_t* kModelExplorerCommandTitle = L"Generate 3D Model Image";

std::string GetLowerExtension(const std::filesystem::path& filePath)
{
    std::string extension = filePath.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension;
}

const char* GetPathKindLabel(const std::filesystem::path& filePath)
{
    return std::filesystem::is_directory(filePath) ? "directory" : "file";
}
}


ModelMenu::ModelMenu() : m_ObjRefCount(1)
{
    DllAddRef();
}

ModelMenu::~ModelMenu()
{
    DllRelease();
}

bool ModelMenu::IsSupportedModelPath(const std::filesystem::path& path) const
{
    if (std::filesystem::is_directory(path))
    {
        return true;
    }

    for (const std::string& modelExtension : NCrewsImageGen::ModelFileExtensions)
    {
        if (!path.extension().compare(modelExtension))
        {
            return true;
        }
    }

    return false;
}

HRESULT ModelMenu::CollectFilePathsFromShellItemArray(IShellItemArray* items, std::vector<std::filesystem::path>& outFilePaths) const
{
    if (!items)
    {
        return E_INVALIDARG;
    }

    outFilePaths.clear();

    DWORD itemCount = 0;
    HRESULT hr = items->GetCount(&itemCount);
    if (FAILED(hr))
    {
        return hr;
    }

    for (DWORD index = 0; index < itemCount; ++index)
    {
        IShellItem* shellItem = nullptr;
        hr = items->GetItemAt(index, &shellItem);
        if (FAILED(hr) || !shellItem)
        {
            continue;
        }

        LPWSTR nameBuffer = nullptr;
        hr = shellItem->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &nameBuffer);
        if (SUCCEEDED(hr) && nameBuffer && wcslen(nameBuffer) > 0)
        {
            std::filesystem::path itemPath = nameBuffer;
            if (IsSupportedModelPath(itemPath))
            {
                outFilePaths.push_back(itemPath);
                utility::LogInfo("Model: CollectFilePaths accepted {} path={} extension={}",
                                 GetPathKindLabel(itemPath),
                                 itemPath.string().c_str(),
                                 GetLowerExtension(itemPath).c_str());
            }
            else
            {
                utility::LogInfo("Model: CollectFilePaths ignored unsupported path={} extension={}",
                                 itemPath.string().c_str(),
                                 GetLowerExtension(itemPath).c_str());
            }
        }

        if (nameBuffer)
        {
            CoTaskMemFree(nameBuffer);
        }

        shellItem->Release();
    }

    if (outFilePaths.empty())
    {
        utility::LogInfo("Model: CollectFilePaths found no supported model files in the current selection.");
        return E_FAIL;
    }

    return S_OK;
}

HRESULT ModelMenu::RenderFilePaths(const std::vector<std::filesystem::path>& filePaths)
{
    if (filePaths.empty())
    {
        return E_INVALIDARG;
    }
    utility::LogInfo("Model: RenderFilePaths begin with {} selected path(s).", filePaths.size());

    std::vector<std::filesystem::path> filesToImage;
    filesToImage.reserve(filePaths.size());
    for (const std::filesystem::path& filePath : filePaths)
    {
        filesToImage.push_back(filePath);
        utility::LogInfo("Model: Right-click render attempt queued path={} extension={}",
                         filePath.string().c_str(),
                         GetLowerExtension(filePath).c_str());
    }

    std::filesystem::path settingsFilePath = g_AppDataPath;
    settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());

    if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
    {
        utility::LogInfo("Model: Right-click render failed to load settings from {}", settingsFilePath.string().c_str());
    }

    tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> renderResults;
    UINT renderStatus = NCrewsImageGen::RenderModelsToImages(g_AppPath, filesToImage, m_imageGenSettings, renderResults);
    utility::LogInfo("Model: RenderFilePaths completed render call (status={}, result_count={})",
                     renderStatus,
                     renderResults.size());
    for (const NCrewsImageGen::FileProcessPackage& renderResult : renderResults)
    {
        utility::LogInfo("Model: Right-click render result file={} extension={} cache_hit={} output={} duration_seconds={}",
                         renderResult.m_FileName.string().c_str(),
                         GetLowerExtension(renderResult.m_FileName).c_str(),
                         renderResult.m_imageFileCacheOk ? "true" : "false",
                         renderResult.m_ImageName.string().c_str(),
                         renderResult.m_processTimeSeconds);
    }

    return S_OK;
}

// IShellExtInit
HRESULT ModelMenu::Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID)
{
    HRESULT hr = E_FAIL;
    IShellItemArray* items = nullptr;

    try
    {
        open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

        if (!pdtobj)
        {
            return E_INVALIDARG;
        }

        hr = SHCreateShellItemArrayFromDataObject(pdtobj, IID_IShellItemArray, (void**)&items);
        if (FAILED(hr) || !items)
        {
            return hr;
        }

        utility::LogInfo("Model: Initialize Context Menu...");
        hr = CollectFilePathsFromShellItemArray(items, m_filePaths);

        utility::LogInfo("Model: Initialize Context Menu...finished");
    }
    catch (...)
    {
        utility::LogInfo("Model: Initialize Context Menu....catch!");
        hr = E_FAIL;
    }

    if (items)
    {
        items->Release();
    }

    return hr;
}

IFACEMETHODIMP ModelMenu::GetTitle(IShellItemArray* psiItemArray, LPWSTR* ppszName)
{
    UNREFERENCED_PARAMETER(psiItemArray);
    if (!ppszName)
    {
        return E_INVALIDARG;
    }

    *ppszName = nullptr;
    return SHStrDupW(kModelExplorerCommandTitle, ppszName);
}

IFACEMETHODIMP ModelMenu::GetIcon(IShellItemArray* psiItemArray, LPWSTR* ppszIcon)
{
    UNREFERENCED_PARAMETER(psiItemArray);
    if (!ppszIcon)
    {
        return E_INVALIDARG;
    }

    *ppszIcon = nullptr;
    return E_NOTIMPL;
}

IFACEMETHODIMP ModelMenu::GetToolTip(IShellItemArray* psiItemArray, LPWSTR* ppszInfotip)
{
    UNREFERENCED_PARAMETER(psiItemArray);
    if (!ppszInfotip)
    {
        return E_INVALIDARG;
    }

    *ppszInfotip = nullptr;
    return E_NOTIMPL;
}

IFACEMETHODIMP ModelMenu::GetCanonicalName(GUID* pguidCommandName)
{
    if (!pguidCommandName)
    {
        return E_INVALIDARG;
    }

    *pguidCommandName = ModelMenuGUID;
    return S_OK;
}

IFACEMETHODIMP ModelMenu::GetState(IShellItemArray* psiItemArray, BOOL fOkToBeSlow, EXPCMDSTATE* pCmdState)
{
    UNREFERENCED_PARAMETER(fOkToBeSlow);
    if (!pCmdState)
    {
        return E_INVALIDARG;
    }

    *pCmdState = ECS_HIDDEN;
    if (!psiItemArray)
    {
        return S_OK;
    }

    std::vector<std::filesystem::path> selectedFilePaths;
    HRESULT hr = CollectFilePathsFromShellItemArray(psiItemArray, selectedFilePaths);
    if (SUCCEEDED(hr) && !selectedFilePaths.empty())
    {
        *pCmdState = ECS_ENABLED;
    }

    return S_OK;
}

IFACEMETHODIMP ModelMenu::Invoke(IShellItemArray* psiItemArray, IBindCtx* pbc)
{
    UNREFERENCED_PARAMETER(pbc);
    open3d::utility::Logger::GetInstance().SetPrintFunction(model_print_fcn);

    if (!psiItemArray)
    {
        utility::LogInfo("Model: ExplorerCommand Invoke missing selection.");
        return E_INVALIDARG;
    }

    std::vector<std::filesystem::path> selectedFilePaths;
    HRESULT hr = CollectFilePathsFromShellItemArray(psiItemArray, selectedFilePaths);
    if (FAILED(hr) || selectedFilePaths.empty())
    {
        utility::LogInfo("Model: ExplorerCommand Invoke has no supported files.");
        return E_FAIL;
    }

    utility::LogInfo("Model: ExplorerCommand Invoke called....");
    utility::LogInfo("Model: ExplorerCommand Invoke supported file count={}", selectedFilePaths.size());
    hr = RenderFilePaths(selectedFilePaths);
    utility::LogInfo("Model: ExplorerCommand Invoke called....Finished");
    return hr;
}

IFACEMETHODIMP ModelMenu::GetFlags(EXPCMDFLAGS* pFlags)
{
    if (!pFlags)
    {
        return E_INVALIDARG;
    }

    *pFlags = ECF_DEFAULT;
    return S_OK;
}

IFACEMETHODIMP ModelMenu::EnumSubCommands(IEnumExplorerCommand** ppEnum)
{
    if (!ppEnum)
    {
        return E_INVALIDARG;
    }

    *ppEnum = nullptr;
    return E_NOTIMPL;
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

            std::wstring menuItemName = L"Generate 3D Model Image";

            if (m_filePaths.size() > 1)
            {
                WCHAR fileCountStr[MAX_PATH] = { 0 };
                _swprintf(fileCountStr, L"%zd", m_filePaths.size());

                menuItemName = L"Generate " + std::wstring(fileCountStr) + L" 3D Model Images";
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
        utility::LogInfo("Model: Menu Invoke Command supported file count={}", m_filePaths.size());
        hr = RenderFilePaths(m_filePaths);

        m_filePaths.clear();

        utility::LogInfo("Model: Menu Invoke Command called....Finished");
    }
    catch (...)
    {
        utility::LogInfo("Model: Menu Invoke Command....catch!");
    }

    return hr;
}

