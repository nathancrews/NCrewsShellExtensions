#pragma once
#include "Renderers/RenderToImageCommon.h"


namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderModelsToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths, 
                                      tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
NCRAFTIMAGEGENAPI HBITMAP RenderModelToHBITMAP(std::filesystem::path& appPath, std::filesystem::path& filePath);

NCRAFTIMAGEGENAPI UINT RenderModelToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo);
}
