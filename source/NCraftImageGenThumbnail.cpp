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
#include "NCraftImageGen.h"
#include "NCraftImageGenThumbnail.h"
#include "Renderers/RenderGLTFToImage.h"


NCraftImageGenThumbnail::NCraftImageGenThumbnail() : m_ObjRefCount(1)
{
    DllAddRef();
}

NCraftImageGenThumbnail::~NCraftImageGenThumbnail()
{
    DllRelease();
}

STDMETHODIMP NCraftImageGenThumbnail::GetSite(REFIID riid, void** ppvSite)
{
    utility::LogInfo("GetSite called...");
    if (m_pSite)
    {
        return m_pSite->QueryInterface(riid, ppvSite);
    }
    return E_NOINTERFACE;
}

STDMETHODIMP NCraftImageGenThumbnail::SetSite(IUnknown* pUnkSite)
{
    utility::LogInfo("SetSite called...");
    if (m_pSite)
    {
        m_pSite->Release();
        m_pSite = NULL;
    }

    m_pSite = pUnkSite;
    if (m_pSite)
    {
        m_pSite->AddRef();
    }
    return S_OK;
}


STDMETHODIMP NCraftImageGenThumbnail::Initialize(IStream* pstream, DWORD grfMode)
{
    ULONG len = 0;
    STATSTG stat;
    WCHAR  tempFileName[MAX_PATH] = { 0 };

    m_filePath = std::filesystem::temp_directory_path();

    utility::LogInfo("Initialize With Stream called {}", m_filePath.string().c_str());

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
            FILE* myfile = nullptr;

            if (GetTempFileNameW(m_filePath.c_str(), L"thumb", 0, tempFileName))
            {
                DeleteFile(tempFileName);

                m_filePath = m_filePath.append(tempFileName);
                m_filePath = m_filePath.replace_extension(L".glb");

                myfile = fopen(m_filePath.string().c_str(), "wb+");

                if (myfile)
                {
                    int byteOut = fwrite(data, len, 1, myfile);

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

    return S_OK;
}

STDMETHODIMP NCraftImageGenThumbnail::GetThumbnail(UINT flag, HBITMAP* outHBITMAP, WTS_ALPHATYPE* alphaType)
{
    HRESULT res = E_FAIL;

    utility::LogInfo("GetThumbnail called...");

    HBITMAP localBMP = nullptr;

    localBMP = NCraftImageGen::RenderModelToHBITMAP(g_AppPath, m_filePath);

    if (localBMP)
    {
        *outHBITMAP = localBMP;

        res = S_OK;
    }

    std::filesystem::remove(m_filePath);

    return res;
}

STDMETHODIMP NCraftImageGenThumbnail::Initialize(LPCWSTR pszFilePath, DWORD grfMode)
{
    utility::LogInfo("Initialize With File called...");
    return S_OK;
}

STDMETHODIMP NCraftImageGenThumbnail::Initialize(IShellItem* psi, DWORD grfMode)
{
    utility::LogInfo("Initialize With IShellItem called...");
    return S_OK;
}



