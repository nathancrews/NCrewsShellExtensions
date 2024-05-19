#pragma once
#include "RenderToImageCommon.h"


namespace NCrewsImageGen
{

NCRAFTIMAGEGENAPI UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                             NCrewsImageGen::AppSettings& imageSettings,
                                             tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults);

NCRAFTIMAGEGENAPI void SetCameraEyeAndTarget(Open3DScene* scene);

NCRAFTIMAGEGENAPI UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCrewsImageGen::AppSettings& imageSettings, 
                                               NCrewsImageGen::FileProcessPackage& fileInfo);

NCRAFTIMAGEGENAPI UINT RenderPointcloudFilesToSingleImage(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                                          NCrewsImageGen::AppSettings& imageSettings, 
                                                          tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults);
NCRAFTIMAGEGENAPI UINT RenderPointcloudsToImage(FilamentRenderer* modelRenderer, NCrewsImageGen::AppSettings& imageSettings,
                                                tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& fileInfos);

}

