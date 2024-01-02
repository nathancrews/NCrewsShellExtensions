#pragma once

#include "priv.h"

class CloudClassFactory : public IClassFactory
{
public:
    CloudClassFactory();
    
    ULONG STDMETHODCALLTYPE AddRef(void);
    ULONG STDMETHODCALLTYPE Release();
    HRESULT STDMETHODCALLTYPE QueryInterface(_In_ REFIID riid, _COM_Outptr_ void** ppvObject);

    HRESULT STDMETHODCALLTYPE CreateInstance(_In_opt_ IUnknown* pUnkOuter, _In_ REFIID riid, _COM_Outptr_ void** ppvObject);
    HRESULT STDMETHODCALLTYPE LockServer(BOOL fLock);

protected:
    DWORD m_ObjRefCount;
    ~CloudClassFactory();
};

