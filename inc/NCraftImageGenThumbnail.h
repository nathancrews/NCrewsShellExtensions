﻿////////////////////////////////////////////////////////////////////////////////////
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
#pragma once

#include "priv.h"
#include "NCraftClassFactory.h"
#include "thumbcache.h"
#include "propsys.h"
#include "ocidl.h"

class NCraftImageGenThumbnail : public IThumbnailProvider, IInitializeWithStream, IObjectWithSite
{
public:

    NCraftImageGenThumbnail();

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
    ~NCraftImageGenThumbnail();

    long        m_ObjRefCount = 0;
    std::filesystem::path  m_filePath;
    UINT m_idCmdFirst = 0;
    IUnknown* m_pSite = nullptr;
};
