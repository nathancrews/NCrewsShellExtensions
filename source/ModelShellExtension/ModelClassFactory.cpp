////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan Crews, NCrews Software
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

    if (IsEqualIID(riid, IID_IThumbnailProvider) || IsEqualIID(riid, IID_IInitializeWithStream))
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

