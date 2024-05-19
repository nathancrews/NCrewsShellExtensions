#pragma once
#include "Renderers/RenderToImageCommon.h"

#include <objidl.h>
#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")


namespace NCrewsImageGen
{

NCRAFTIMAGEGENAPI UINT RenderModelsToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                            NCrewsImageGen::AppSettings& imageSettings,
                                            tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults);
NCRAFTIMAGEGENAPI HBITMAP RenderModelToHBITMAP(std::filesystem::path& appPath,
                                               NCrewsImageGen::AppSettings& imageSettings, std::filesystem::path& filePath);

NCRAFTIMAGEGENAPI UINT RenderModelToImage(FilamentRenderer* modelRenderer,
                                          NCrewsImageGen::AppSettings& imageSettings, NCrewsImageGen::FileProcessPackage& fileInfo);

NCRAFTIMAGEGENAPI int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);
}
