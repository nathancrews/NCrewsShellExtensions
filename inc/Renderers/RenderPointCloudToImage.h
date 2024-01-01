#pragma once
#include "RenderToImageCommon.h"

namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                                tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo);
UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outLoadResults);

}

