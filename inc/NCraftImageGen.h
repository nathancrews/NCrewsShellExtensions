#pragma once
#include "priv.h"

STDAPI_(BOOL) DllMain(HINSTANCE hInstance, DWORD dwReason, void*);
STDAPI DllCanUnloadNow(void);
void __stdcall DllAddRef(void);
void __stdcall DllRelease(void);
STDAPI DllGetClassObject(_In_ REFCLSID clsid, _In_ REFIID riid, _Outptr_ LPVOID* ppv);
HRESULT __stdcall  DllRegisterServer();
HRESULT __stdcall  DllUnregisterServer();
