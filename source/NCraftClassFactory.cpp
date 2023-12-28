#include "NCraftImageGen.h"
#include "NCraftClassFactory.h"
#include "NCraftImageGenMenu.h"


NCraftClassFactory::NCraftClassFactory() : m_ObjRefCount(1)
{
    InterlockedIncrement(&g_DllModuleRefCount);
}

NCraftClassFactory::~NCraftClassFactory()
{
    InterlockedDecrement(&g_DllModuleRefCount);
}

ULONG NCraftClassFactory::AddRef(void)
{
    InterlockedIncrement(&m_ObjRefCount);
    return m_ObjRefCount;
}

ULONG NCraftClassFactory::Release(void)
{
    ULONG retVal = InterlockedDecrement(&m_ObjRefCount);

    if (retVal < 1)
    {
        delete this;
    }

    return retVal;
}

HRESULT NCraftClassFactory::QueryInterface(_In_ REFIID riid, _COM_Outptr_ void** ppvObject)
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

HRESULT NCraftClassFactory::CreateInstance(_In_opt_ IUnknown* pUnkOuter, _In_ REFIID riid, _COM_Outptr_ void** ppvObject)
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

    if (IsEqualIID(riid, IID_IShellExtInit) || IsEqualIID(riid, IID_IContextMenu))
    {
        NCraftImageGenContextMenu* cMenu = new NCraftImageGenContextMenu();
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

HRESULT NCraftClassFactory::LockServer(BOOL fLock)
{
    return E_NOTIMPL;
}



