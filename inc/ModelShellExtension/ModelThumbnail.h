#pragma once

#include "priv.h"
#include "ModelClassFactory.h"
#include "thumbcache.h"
#include "propsys.h"
#include "ocidl.h"

class ModelThumbnail : public IThumbnailProvider, IInitializeWithStream, IObjectWithSite
{
public:

    ModelThumbnail();

    // IUnknown methods
    IFACEMETHODIMP_(ULONG) AddRef()
    {
        return InterlockedIncrement(&m_ObjRefCount);
    }

    IFACEMETHODIMP_(ULONG) Release()
    {
        long cRef = InterlockedDecrement(&m_ObjRefCount);
        if (cRef == 0)
        {
            delete this;
        }
        return cRef;
    }
    IFACEMETHODIMP QueryInterface(REFIID riid, void** ppvObject)
    {
        if (!ppvObject)
        {
            return E_INVALIDARG;
        }

        *ppvObject = nullptr;

        if (IsEqualIID(riid, IID_IUnknown))
        {
            *ppvObject = this;
            this->AddRef();
            return S_OK;
        }
        else if (IsEqualIID(riid, IID_IThumbnailProvider))
        {
            *ppvObject = (IThumbnailProvider*)this;
            this->AddRef();
            return S_OK;
        }
        else if (IsEqualIID(riid, IID_IInitializeWithFile))
        {
            *ppvObject = (IInitializeWithFile*)this;
            this->AddRef();
            return S_OK;
        }
        else if (IsEqualIID(riid, IID_IInitializeWithItem))
        {
            *ppvObject = (IInitializeWithItem*)this;
            this->AddRef();
            return S_OK;
        }
        else if (IsEqualIID(riid, IID_IInitializeWithStream))
        {
            *ppvObject = (IInitializeWithStream*)this;
            this->AddRef();
            return S_OK;
        }

        else
        {
            return E_NOINTERFACE;
        }
    }

    //  IInitializeWithSteam methods
    STDMETHOD(Initialize)(IStream* pstream, DWORD grfMode);
    STDMETHOD(Initialize)(IShellItem* psi, DWORD grfMode);
    STDMETHOD(Initialize)(LPCWSTR pszFilePath, DWORD grfMode);

    //  IThumbnailProvider methods
    STDMETHOD(GetThumbnail)(UINT, HBITMAP*, WTS_ALPHATYPE*);

    STDMETHOD(GetSite)(REFIID, void**);
    STDMETHOD(SetSite)(IUnknown*);

private:
    ~ModelThumbnail();

    long        m_ObjRefCount = 0;
    std::filesystem::path  m_filePath;
    UINT m_idCmdFirst = 0;
    IUnknown* m_pSite = nullptr;
};
