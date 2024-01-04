#pragma once
#include "RenderToImageCommon.h"

namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                                tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
NCRAFTIMAGEGENAPI UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo);
UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPLYToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPTSToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadXYZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);

UINT LoadPointCloudFile(NCraftImageGen::ImageGenResult& outLoadResults);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outLoadResults);

}

