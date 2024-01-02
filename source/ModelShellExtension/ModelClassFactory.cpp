#include "ModelShellExtension.h"
#include "ModelClassFactory.h"
#include "ModelMenu.h"
#include "ModelThumbnail.h"


ModelClassFactory::ModelClassFactory() : m_ObjRefCount(1)
{
    InterlockedIncrement(&g_DllModuleRefCount);
}

ModelClassFactory::~ModelClassFactory()
{
    InterlockedDecrement(&g_DllModuleRefCount);
}

ULONG ModelClassFactory::AddRef(void)
{
    InterlockedIncrement(&m_ObjRefCount);
    return m_ObjRefCount;
}

ULONG ModelClassFactory::Release(void)
{
    ULONG retVal = InterlockedDecrement(&m_ObjRefCount);

    if (retVal < 1)
    {
        delete this;
    }

    return retVal;
}

HRESULT ModelClassFactory::QueryInterface(_In_ REFIID riid, _COM_Outptr_ void** ppvObject)
{
    if (!ppvObject)
    {
        return E_INVALIDARG;
    }

    *ppvObject = nullptr;

    if (IsEqualIID(riid, IID_IUnknown))
    {
        *ppvObject = (IUnknown*)this;
        this->AddRef();
        return S_OK;
    }
    else if (IsEqualIID(riid, IID_IClassFactory))
    {
        *ppvObject = (IClassFactory*)this;
        this->AddRef();
        return S_OK;
    }
    else
    {
        return E_NOINTERFACE;
    }
}

HRESULT ModelClassFactory::CreateInstance(_In_opt_ IUnknown* pUnkOuter, _In_ REFIID riid, _COM_Outptr_ void** ppvObject)
{

    if (!ppvObject)
    {
        return E_INVALIDARG;
    }
 
    if (pUnkOuter != nullptr)
    {
        return CLASS_E_NOAGGREGATION;
    }

    *ppvObject = nullptr;

    HRESULT hres = E_NOINTERFACE;

    if (IsEqualIID(riid, IID_IThumbnailProvider) || IsEqualIID(riid, IID_IInitializeWithStream) ||
        IsEqualIID(riid, IID_IInitializeWithFile) ||
        IsEqualIID(riid, IID_IInitializeWithItem))
    {
        ModelThumbnail* cThumb = new ModelThumbnail();
        if (cThumb)
        {
            hres = cThumb->QueryInterface(riid, ppvObject);
            cThumb->Release();
        }
        else
        {
            hres = E_OUTOFMEMORY;
        }
    }

    if (IsEqualIID(riid, IID_IShellExtInit) || IsEqualIID(riid, IID_IContextMenu))
    {
        ModelMenu* cMenu = new ModelMenu();
        if (cMenu)
        {
            hres = cMenu->QueryInterface(riid, ppvObject);
            cMenu->Release();
        }
        else
        {
            hres = E_OUTOFMEMORY;
        }
    }

    return hres;
}

HRESULT ModelClassFactory::LockServer(BOOL fLock)
{
    return E_NOTIMPL;
}

