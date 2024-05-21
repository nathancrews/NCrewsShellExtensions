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


#pragma once

#include "priv.h"
#include "CloudClassFactory.h"
#include "Renderers/RenderToImageCommon.h"

class CloudMenu : public IContextMenu,  public IShellExtInit
{
public:

    CloudMenu();

    // IUnknown methods

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
        else if (IsEqualIID(riid, IID_IContextMenu))
        {
            *ppvObject = (IContextMenu*)this;
            this->AddRef();
            return S_OK;
        }
        else if (IsEqualIID(riid, IID_IShellExtInit))
        {
            *ppvObject = (IShellExtInit*)this;
            this->AddRef();
            return S_OK;
        }
        else
        {
            return E_NOINTERFACE;
        }
    }

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

    // IContextMenu
    HRESULT __stdcall QueryContextMenu(HMENU hmenu, UINT indexMenu, UINT idCmdFirst, UINT idCmdLast, UINT uFlags);
    HRESULT __stdcall InvokeCommand(LPCMINVOKECOMMANDINFO lpici);
    HRESULT __stdcall GetCommandString(UINT_PTR /*idCmd*/, UINT /*uType*/, UINT* /*pRes*/, LPSTR /*pszName*/, UINT /*cchMax*/) { return E_NOTIMPL; }

    // IShellExtInit
    IFACEMETHODIMP Initialize(PCIDLIST_ABSOLUTE pidlFolder, IDataObject* pdtobj, HKEY hkeyProgID);

private:
    ~CloudMenu();

    long        m_ObjRefCount = 0;
    std::vector<std::filesystem::path>  m_filePaths;
    UINT m_idCmdFirst = 0;
    NCrewsImageGen::AppSettings m_imageGenSettings;


};
