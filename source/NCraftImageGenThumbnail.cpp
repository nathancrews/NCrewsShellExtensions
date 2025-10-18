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



