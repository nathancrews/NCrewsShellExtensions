#pragma once

#include "priv.h"
#include "NCraftClassFactory.h"

class NCraftImageGenContextMenu : public IContextMenu,  public IShellExtInit
{
public:

    NCraftImageGenContextMenu();

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
    ~NCraftImageGenContextMenu();

    long        m_ObjRefCount = 0;
    std::vector<std::wstring>  m_filePaths;
    UINT m_idCmdFirst = 0;

};
