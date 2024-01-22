#pragma once
#include "Renderers/RenderToImageCommon.h"

#include <objidl.h>
#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")


namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderModelsToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                            NCraftImageGen::ImageGenSettings& imageSettings,
                                            tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
NCRAFTIMAGEGENAPI HBITMAP RenderModelToHBITMAP(std::filesystem::path& appPath,
                                               NCraftImageGen::ImageGenSettings& imageSettings, std::filesystem::path& filePath);

NCRAFTIMAGEGENAPI UINT RenderModelToImage(FilamentRenderer* modelRenderer,
                                          NCraftImageGen::ImageGenSettings& imageSettings, NCraftImageGen::ImageGenResult& fileInfo);

NCRAFTIMAGEGENAPI int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);
}
