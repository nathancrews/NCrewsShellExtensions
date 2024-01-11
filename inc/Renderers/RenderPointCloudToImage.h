#pragma once
#include "RenderToImageCommon.h"

namespace pcl
{
    struct PointXYZRGB;
}

namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                             NCraftImageGen::ImageGenSettings& imageSettings,
                                             tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
NCRAFTIMAGEGENAPI UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenSettings& imageSettings, 
                                               NCraftImageGen::ImageGenResult& fileInfo);

NCRAFTIMAGEGENAPI UINT RenderPointcloudFilesToSingleImage(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                                          NCraftImageGen::ImageGenSettings& imageSettings, 
                                                          tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
NCRAFTIMAGEGENAPI UINT RenderPointcloudsToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenSettings& imageSettings,
                                                tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& fileInfos);

UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud, 
                            pcl::PointXYZRGB* pCommonBasePoint);
UINT LoadPLYToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPTSToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadXYZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);

UINT LoadPointCloudFile(NCraftImageGen::ImageGenResult& outLoadResults, pcl::PointXYZRGB* pCommonBasePoint);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outLoadResults);

}

