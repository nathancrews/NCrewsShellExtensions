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
#include "ModelThumbnail.h"
#include "Renderers/RenderGLTFToImage.h"
#include <cstdint>
#include <cctype>
#include <cstring>
#include <algorithm>
#include <cwctype>


ModelThumbnail::ModelThumbnail() : m_ObjRefCount(1)
{
    DllAddRef();
}

ModelThumbnail::~ModelThumbnail()
{
    DllRelease();
}

STDMETHODIMP ModelThumbnail::Initialize(IStream* pstream, DWORD grfMode)
{
    ULONG len = 0;
    STATSTG stat;
    WCHAR  tempFileName[MAX_PATH] = { 0 };
    std::wstring inferredExt = L".glb";

    try
    {
        m_filePath = std::filesystem::temp_directory_path();

        if (pstream->Stat(&stat, STATFLAG_DEFAULT) != S_OK)
        {
            utility::LogInfo("bad Stat, returning early....");
            return S_FALSE;
        }

        if ((stat.cbSize.QuadPart == 0))
        {
            utility::LogInfo("bad file size....");
            return S_FALSE;
        }

        // get the file contents
        char* data = nullptr;

        data = new char[stat.cbSize.QuadPart];

        if (data)
        {
            if (pstream->Read(data, stat.cbSize.QuadPart, &len) == S_OK)
            {
                // Infer extension for stream-backed thumbnails to support STL and GLTF/GLB.
                if (len >= 4 && std::memcmp(data, "glTF", 4) == 0)
                {
                    inferredExt = L".glb";
                }
                else
                {
                    size_t firstNonWhitespace = 0;
                    while (firstNonWhitespace < len &&
                           std::isspace(static_cast<unsigned char>(data[firstNonWhitespace])))
                    {
                        ++firstNonWhitespace;
                    }

                    if (firstNonWhitespace < len && data[firstNonWhitespace] == '{')
                    {
                        inferredExt = L".gltf";
                    }
                    else
                    {
                        bool looksAsciiStl = false;
                        if (len >= 5)
                        {
                            looksAsciiStl =
                                std::tolower(static_cast<unsigned char>(data[0])) == 's' &&
                                std::tolower(static_cast<unsigned char>(data[1])) == 'o' &&
                                std::tolower(static_cast<unsigned char>(data[2])) == 'l' &&
                                std::tolower(static_cast<unsigned char>(data[3])) == 'i' &&
                                std::tolower(static_cast<unsigned char>(data[4])) == 'd';
                        }

                        bool looksBinaryStl = false;
                        if (len >= 84)
                        {
                            std::uint32_t triCount = 0;
                            std::memcpy(&triCount, data + 80, sizeof(std::uint32_t));
                            std::uint64_t expectedSize = 84ULL + (static_cast<std::uint64_t>(triCount) * 50ULL);
                            looksBinaryStl = expectedSize == static_cast<std::uint64_t>(len);
                        }

                        if (looksAsciiStl || looksBinaryStl)
                        {
                            inferredExt = L".stl";
                        }
                    }
                }
                FILE* myfile = nullptr;

                if (GetTempFileNameW(m_filePath.c_str(), L"thumb", 0, tempFileName))
                {
                    DeleteFile(tempFileName);

                    m_filePath = m_filePath.append(tempFileName);
                    m_filePath = m_filePath.replace_extension(inferredExt);

                    myfile = fopen(m_filePath.string().c_str(), "wb+");

                    if (myfile)
                    {
                        size_t byteOut = fwrite(data, len, 1, myfile);

                        fclose(myfile);
                    }
                }
            }
            else
            {
                utility::LogInfo("read failed....");
            }

            delete[] data;
        }

        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());

        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings");
        }

        return S_OK;
    }
    catch (...)
    {
        return E_FAIL;
    }
}

STDMETHODIMP ModelThumbnail::GetThumbnail(UINT flag, HBITMAP* outHBITMAP, WTS_ALPHATYPE* alphaType)
{
    HRESULT res = E_FAIL;

    utility::LogInfo("GetThumbnail called...");
    
    if (!outHBITMAP)
    {
        return E_INVALIDARG;
    }
    
    *outHBITMAP = nullptr;
    if (alphaType)
    {
        *alphaType = WTSAT_RGB;  // No alpha channel in our thumbnails
    }
    
    try
    {
        HBITMAP localBMP = nullptr;
        
        // Check if we have a valid file path
        if (m_filePath.empty() || !std::filesystem::exists(m_filePath))
        {
            utility::LogInfo("Invalid or missing file path");
            return E_FAIL;
        }

        std::wstring fileExt = m_filePath.extension().wstring();
        std::transform(fileExt.begin(), fileExt.end(), fileExt.begin(),
            [](wchar_t ch) { return static_cast<wchar_t>(std::towlower(ch)); });

        if (fileExt == L".gltf")
        {
            utility::LogInfo("GLTF thumbnail requests are intentionally unsupported due to external file dependencies.");
            return E_FAIL;
        }

        localBMP = NCrewsImageGen::RenderModelToHBITMAP(g_AppPath, m_imageGenSettings, m_filePath);

        if (localBMP)
        {
            *outHBITMAP = localBMP;
            res = S_OK;
        }
        else
        {
            utility::LogInfo("Failed to render model to bitmap");
        }

        // Only remove temp files (those in temp directory)
        if (m_filePath.string().find(std::filesystem::temp_directory_path().string()) != std::string::npos)
        {
            std::filesystem::remove(m_filePath);
        }
    }
    catch (const std::exception& e)
    {
        std::string errorMsg = "Exception in GetThumbnail: " + std::string(e.what());
        utility::LogInfo(errorMsg.c_str());
        res = E_FAIL;
    }
    catch (...)
    {
        utility::LogInfo("Unknown exception in GetThumbnail");
        res = E_FAIL;
    }

    return res;
}

STDMETHODIMP ModelThumbnail::Initialize(LPCWSTR pszFilePath, DWORD grfMode)
{
    utility::LogInfo("Initialize With File called...");
    
    try
    {
        if (!pszFilePath)
        {
            return E_INVALIDARG;
        }
        
        m_filePath = pszFilePath;
        
        // Load settings
        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());
        
        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings");
        }
        
        return S_OK;
    }
    catch (...)
    {
        return E_FAIL;
    }
}

STDMETHODIMP ModelThumbnail::Initialize(IShellItem* psi, DWORD grfMode)
{
    utility::LogInfo("Initialize With IShellItem called...");
    
    try
    {
        if (!psi)
        {
            return E_INVALIDARG;
        }
        
        LPWSTR pszFilePath = nullptr;
        HRESULT hr = psi->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
        
        if (SUCCEEDED(hr) && pszFilePath)
        {
            m_filePath = pszFilePath;
            CoTaskMemFree(pszFilePath);
            
            // Load settings
            std::filesystem::path settingsFilePath = g_AppDataPath;
            settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());
            
            if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
            {
                utility::LogInfo("Error loading settings");
            }
            
            return S_OK;
        }
        
        return hr;
    }
    catch (...)
    {
        return E_FAIL;
    }
}



