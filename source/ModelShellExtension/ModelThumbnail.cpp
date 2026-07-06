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
#include <string>
#include <sstream>
#include <iomanip>
namespace
{
std::string ToLogString(const std::wstring& value)
{
    return std::string(value.begin(), value.end());
}

std::wstring ToLowerWString(const std::wstring& value)
{
    std::wstring lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(),
        [](wchar_t ch) { return static_cast<wchar_t>(std::towlower(ch)); });
    return lowered;
}

bool IsSupportedModelExtension(const std::wstring& fileExtension)
{
    const std::wstring extension = ToLowerWString(fileExtension);
    return (extension == L".glb") || (extension == L".gltf") || (extension == L".stl") ||
           (extension == L".obj") || (extension == L".3mf");
}

size_t SkipUtf8Bom(const char* data, const size_t dataLength)
{
    if (!data)
    {
        return 0;
    }

    if ((dataLength >= 3) &&
        (static_cast<unsigned char>(data[0]) == 0xEF) &&
        (static_cast<unsigned char>(data[1]) == 0xBB) &&
        (static_cast<unsigned char>(data[2]) == 0xBF))
    {
        return 3;
    }

    return 0;
}

size_t FindFirstNonWhitespace(const char* data, const size_t dataLength, const size_t startIndex = 0)
{
    size_t index = startIndex;
    while ((index < dataLength) &&
           std::isspace(static_cast<unsigned char>(data[index])))
    {
        ++index;
    }
    return index;
}

bool StartsWithIgnoreCase(const char* data, const size_t dataLength, const char* token)
{
    if (!data || !token)
    {
        return false;
    }

    const size_t tokenLength = std::strlen(token);
    if (dataLength < tokenLength)
    {
        return false;
    }

    for (size_t index = 0; index < tokenLength; ++index)
    {
        const auto dataChar = static_cast<unsigned char>(data[index]);
        const auto tokenChar = static_cast<unsigned char>(token[index]);
        if (std::tolower(dataChar) != std::tolower(tokenChar))
        {
            return false;
        }
    }

    return true;
}

bool ContainsIgnoreCase(const char* data, const size_t dataLength, const char* token)
{
    if (!data || !token)
    {
        return false;
    }

    const size_t tokenLength = std::strlen(token);
    if ((tokenLength == 0) || (dataLength < tokenLength))
    {
        return false;
    }

    for (size_t offset = 0; (offset + tokenLength) <= dataLength; ++offset)
    {
        bool isMatch = true;
        for (size_t tokenIndex = 0; tokenIndex < tokenLength; ++tokenIndex)
        {
            const auto dataChar = static_cast<unsigned char>(data[offset + tokenIndex]);
            const auto tokenChar = static_cast<unsigned char>(token[tokenIndex]);
            if (std::tolower(dataChar) != std::tolower(tokenChar))
            {
                isMatch = false;
                break;
            }
        }

        if (isMatch)
        {
            return true;
        }
    }

    return false;
}

std::string GetLeadingBytesHex(const char* data, const size_t dataLength, const size_t maxBytes = 24)
{
    if (!data || (dataLength == 0))
    {
        return "none";
    }

    const size_t bytesToLog = std::min(maxBytes, dataLength);
    std::ostringstream hexStream;
    hexStream << std::hex << std::setfill('0');

    for (size_t index = 0; index < bytesToLog; ++index)
    {
        if (index > 0)
        {
            hexStream << " ";
        }
        hexStream << std::setw(2)
                  << static_cast<unsigned int>(static_cast<unsigned char>(data[index]));
    }

    if (dataLength > bytesToLog)
    {
        hexStream << " ...";
    }

    return hexStream.str();
}

bool LineStartsWithObjToken(const char* lineData, const size_t lineLength)
{
    if (!lineData || (lineLength == 0))
    {
        return false;
    }

    size_t tokenStart = 0;
    while ((tokenStart < lineLength) &&
           ((lineData[tokenStart] == ' ') || (lineData[tokenStart] == '\t')))
    {
        ++tokenStart;
    }

    if (tokenStart >= lineLength)
    {
        return false;
    }

    if (lineData[tokenStart] == '#')
    {
        return false;
    }

    auto startsToken = [&](const char* token) -> bool
    {
        const size_t tokenLength = std::strlen(token);
        if ((lineLength - tokenStart) < tokenLength)
        {
            return false;
        }

        for (size_t index = 0; index < tokenLength; ++index)
        {
            const auto lineChar = static_cast<unsigned char>(lineData[tokenStart + index]);
            const auto tokenChar = static_cast<unsigned char>(token[index]);
            if (std::tolower(lineChar) != std::tolower(tokenChar))
            {
                return false;
            }
        }

        if ((lineLength - tokenStart) == tokenLength)
        {
            return true;
        }

        const char nextChar = lineData[tokenStart + tokenLength];
        return std::isspace(static_cast<unsigned char>(nextChar)) ||
               (nextChar == '/') || (nextChar == '\\');
    };

    return startsToken("v") || startsToken("vn") || startsToken("vt") || startsToken("vp") ||
           startsToken("f") || startsToken("l") || startsToken("o") || startsToken("g") ||
           startsToken("s") || startsToken("mtllib") || startsToken("usemtl");
}

bool LooksLikeObjText(const char* data, const size_t dataLength)
{
    if (!data || (dataLength == 0))
    {
        return false;
    }

    const size_t sampleLength = std::min<size_t>(dataLength, 16384);
    size_t cursor = SkipUtf8Bom(data, sampleLength);
    int inspectedLines = 0;
    constexpr int kMaxInspectedLines = 64;

    while ((cursor < sampleLength) && (inspectedLines < kMaxInspectedLines))
    {
        while ((cursor < sampleLength) &&
               ((data[cursor] == '\r') || (data[cursor] == '\n')))
        {
            ++cursor;
        }

        if (cursor >= sampleLength)
        {
            break;
        }

        const size_t lineStart = cursor;
        while ((cursor < sampleLength) &&
               (data[cursor] != '\r') &&
               (data[cursor] != '\n'))
        {
            ++cursor;
        }
        const size_t lineLength = cursor - lineStart;

        if ((lineLength > 0) &&
            LineStartsWithObjToken(data + lineStart, lineLength))
        {
            return true;
        }

        ++inspectedLines;
    }

    return false;
}

bool LooksLikeAsciiStl(const char* data, const size_t dataLength)
{
    if (!data || (dataLength == 0))
    {
        return false;
    }

    const size_t sampleLength = std::min<size_t>(dataLength, 16384);
    size_t firstMeaningful = SkipUtf8Bom(data, sampleLength);
    firstMeaningful = FindFirstNonWhitespace(data, sampleLength, firstMeaningful);
    if (firstMeaningful >= sampleLength)
    {
        return false;
    }

    const char* sampleStart = data + firstMeaningful;
    const size_t remainingLength = sampleLength - firstMeaningful;
    if (!StartsWithIgnoreCase(sampleStart, remainingLength, "solid"))
    {
        return false;
    }

    return ContainsIgnoreCase(sampleStart, remainingLength, "facet normal") ||
           ContainsIgnoreCase(sampleStart, remainingLength, "endsolid");
}

bool LooksLikeBinaryStl(const char* data, const size_t dataLength, bool& outExactSizeMatch)
{
    outExactSizeMatch = false;
    if (!data || (dataLength < 84))
    {
        return false;
    }

    std::uint32_t triCount = 0;
    std::memcpy(&triCount, data + 80, sizeof(std::uint32_t));
    const std::uint64_t expectedSize = 84ULL + (static_cast<std::uint64_t>(triCount) * 50ULL);
    if (expectedSize == static_cast<std::uint64_t>(dataLength))
    {
        outExactSizeMatch = true;
        return true;
    }

    return ((dataLength - 84ULL) % 50ULL) == 0ULL;
}

std::wstring InferStreamModelExtension(const char* data, const size_t dataLength, std::string& outReason)
{
    outReason = "defaulted to .glb because stream signature was unknown";
    if (!data || (dataLength == 0))
    {
        return L".glb";
    }

    if ((dataLength >= 4) && (std::memcmp(data, "glTF", 4) == 0))
    {
        outReason = "detected GLB binary magic 'glTF'";
        return L".glb";
    }

    if ((dataLength >= 4) &&
        (static_cast<unsigned char>(data[0]) == 0x50) &&
        (static_cast<unsigned char>(data[1]) == 0x4B) &&
        (static_cast<unsigned char>(data[2]) == 0x03) &&
        (static_cast<unsigned char>(data[3]) == 0x04))
    {
        outReason = "detected ZIP signature (PK\\x03\\x04), treating as .3mf";
        return L".3mf";
    }

    size_t firstNonWhitespace = SkipUtf8Bom(data, dataLength);
    firstNonWhitespace = FindFirstNonWhitespace(data, dataLength, firstNonWhitespace);

    if ((firstNonWhitespace < dataLength) && (data[firstNonWhitespace] == '{'))
    {
        outReason = "detected JSON payload, treating as .gltf";
        return L".gltf";
    }
    const bool looksAsciiStl = LooksLikeAsciiStl(data, dataLength);
    bool hasExactBinaryStlMatch = false;
    const bool looksBinaryStl = LooksLikeBinaryStl(data, dataLength, hasExactBinaryStlMatch);

    if (looksAsciiStl || looksBinaryStl)
    {
        if (looksAsciiStl)
        {
            outReason = "detected ASCII STL signature";
        }
        else
        {
            outReason = hasExactBinaryStlMatch ? "detected binary STL layout (exact size match)"
                                               : "detected binary STL layout (50-byte triangle stride)";
        }
        return L".stl";
    }

    if (LooksLikeObjText(data, dataLength))
    {
        outReason = "detected OBJ-style text tokens";
        return L".obj";
    }

    return L".glb";
}
}


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
    if (!pstream)
    {
        utility::LogInfo("Initialize(IStream): null stream pointer.");
        return E_INVALIDARG;
    }
    utility::LogInfo("Initialize(IStream): begin (grfMode={})", static_cast<unsigned long>(grfMode));

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
        utility::LogInfo("Initialize(IStream): received stream size={} bytes", static_cast<unsigned long long>(stat.cbSize.QuadPart));

        // get the file contents
        char* data = nullptr;

        data = new char[stat.cbSize.QuadPart];

        if (data)
        {
            const HRESULT readHr = pstream->Read(data, stat.cbSize.QuadPart, &len);
            if (SUCCEEDED(readHr) && (len > 0))
            {
                utility::LogInfo("Initialize(IStream): leading-byte signature {}",
                                 GetLeadingBytesHex(data, static_cast<size_t>(len)).c_str());
                std::string inferenceReason;
                inferredExt = InferStreamModelExtension(data, static_cast<size_t>(len), inferenceReason);
                utility::LogInfo("Initialize(IStream): inferred extension {} from stream content ({})",
                                 ToLogString(inferredExt).c_str(),
                                 inferenceReason.c_str());
                if ((inferredExt == L".glb") && (inferenceReason.find("defaulted to .glb") != std::string::npos))
                {
                    utility::LogInfo("Initialize(IStream): stream type could not be confidently identified; logs will show .glb fallback.");
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
                        if (byteOut != 1)
                        {
                            utility::LogInfo("Initialize(IStream): failed to write thumbnail temp file {}", m_filePath.string().c_str());
                        }

                        fclose(myfile);
                    }
                    else
                    {
                        utility::LogInfo("Initialize(IStream): could not open temp file for write {}", m_filePath.string().c_str());
                    }
                    utility::LogInfo("Initialize(IStream): temp model path prepared {}", m_filePath.string().c_str());
                }
                else
                {
                    utility::LogInfo("Initialize(IStream): GetTempFileNameW failed (error={})", static_cast<unsigned long>(GetLastError()));
                }
            }
            else
            {
                utility::LogInfo("Initialize(IStream): stream read failed (hr={}, bytesRead={})",
                                 static_cast<long>(readHr),
                                 static_cast<unsigned long>(len));
            }

            delete[] data;
        }
        else
        {
            utility::LogInfo("Initialize(IStream): failed to allocate {} bytes for stream read", static_cast<unsigned long long>(stat.cbSize.QuadPart));
        }

        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());

        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings from {}", settingsFilePath.string().c_str());
        }

        return S_OK;
    }
    catch (const std::exception& e)
    {
        utility::LogInfo("Initialize(IStream): exception {}", e.what());
        return E_FAIL;
    }
    catch (...)
    {
        utility::LogInfo("Initialize(IStream): unknown exception");
        return E_FAIL;
    }
}

STDMETHODIMP ModelThumbnail::GetThumbnail(UINT flag, HBITMAP* outHBITMAP, WTS_ALPHATYPE* alphaType)
{
    HRESULT res = E_FAIL;

    utility::LogInfo("GetThumbnail called (requested size={} px).", flag);
    
    if (!outHBITMAP)
    {
        utility::LogInfo("GetThumbnail: outHBITMAP was null.");
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
            utility::LogInfo("GetThumbnail: invalid or missing file path {}", m_filePath.string().c_str());
            return E_FAIL;
        }

        std::wstring fileExt = m_filePath.extension().wstring();
        std::transform(fileExt.begin(), fileExt.end(), fileExt.begin(),
            [](wchar_t ch) { return static_cast<wchar_t>(std::towlower(ch)); });
        utility::LogInfo("GetThumbnail: thumbnail attempt file={} extension={} requested_size={}px",
                         m_filePath.string().c_str(),
                         ToLogString(fileExt).c_str(),
                         flag);

        if (!IsSupportedModelExtension(fileExt))
        {
            utility::LogInfo("GetThumbnail: extension {} is not in supported set (.glb/.gltf/.stl/.obj/.3mf)",
                             ToLogString(fileExt).c_str());
        }

        utility::LogInfo("GetThumbnail: rendering {} with extension {}", m_filePath.string().c_str(), ToLogString(fileExt).c_str());

        if (fileExt == L".gltf")
        {
            utility::LogInfo("GetThumbnail: GLTF thumbnail requests are intentionally unsupported due to external file dependencies ({}).", m_filePath.string().c_str());
            return E_FAIL;
        }

        localBMP = NCrewsImageGen::RenderModelToHBITMAP(g_AppPath, m_imageGenSettings, m_filePath);

        if (localBMP)
        {
            *outHBITMAP = localBMP;
            res = S_OK;
            utility::LogInfo("GetThumbnail: render succeeded for {}", m_filePath.string().c_str());
        }
        else
        {
            utility::LogInfo("GetThumbnail: render failed for {} (extension={})", m_filePath.string().c_str(), ToLogString(fileExt).c_str());
        }

        // Only remove temp files (those in temp directory)
        if (m_filePath.string().find(std::filesystem::temp_directory_path().string()) != std::string::npos)
        {
            try
            {
                std::filesystem::remove(m_filePath);
            }
            catch (const std::exception& e)
            {
                utility::LogInfo("GetThumbnail: failed to remove temp file {} ({})", m_filePath.string().c_str(), e.what());
            }
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
        const std::wstring fileExt = ToLowerWString(m_filePath.extension().wstring());
        utility::LogInfo("Initialize(IInitializeWithFile): file path {} extension {} grfMode {}",
                         m_filePath.string().c_str(),
                         ToLogString(fileExt).c_str(),
                         static_cast<unsigned long>(grfMode));
        if (!IsSupportedModelExtension(fileExt))
        {
            utility::LogInfo("Initialize(IInitializeWithFile): extension {} is not in supported set (.glb/.gltf/.stl/.obj/.3mf)",
                             ToLogString(fileExt).c_str());
        }
        
        // Load settings
        std::filesystem::path settingsFilePath = g_AppDataPath;
        settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());
        
        if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
        {
            utility::LogInfo("Error loading settings from {}", settingsFilePath.string().c_str());
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
            const std::wstring fileExt = ToLowerWString(m_filePath.extension().wstring());
            utility::LogInfo("Initialize(IInitializeWithItem): file path {} extension {} grfMode {}",
                             m_filePath.string().c_str(),
                             ToLogString(fileExt).c_str(),
                             static_cast<unsigned long>(grfMode));
            if (!IsSupportedModelExtension(fileExt))
            {
                utility::LogInfo("Initialize(IInitializeWithItem): extension {} is not in supported set (.glb/.gltf/.stl/.obj/.3mf)",
                                 ToLogString(fileExt).c_str());
            }
            
            // Load settings
            std::filesystem::path settingsFilePath = g_AppDataPath;
            settingsFilePath = settingsFilePath.concat(g_SettingsFileName.c_str());
            
            if (!NCrewsImageGen::ReadImageGenSettings(settingsFilePath, m_imageGenSettings))
            {
                utility::LogInfo("Error loading settings from {}", settingsFilePath.string().c_str());
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



