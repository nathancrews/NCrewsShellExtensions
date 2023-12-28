#pragma once
#include "framework.h"
#include <filesystem>

#ifdef NCRAFTIMAGEGENAPI_EXPORT
#define NCRAFTIMAGEGENAPI __declspec(dllexport)
#elif defined(LANDXML2GLTFDLLAPI_IMPORT)
#define NCRAFTIMAGEGENAPI __declspec(dllimport)
#else
#define NCRAFTIMAGEGENAPI
#endif

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include "open3d/Open3D.h"
#include "open3d/visualization/rendering/Camera.h"
#include "open3d/visualization/rendering/filament/FilamentEngine.h"
#include "open3d/visualization/rendering/filament/FilamentRenderer.h"
#include "open3d/utility/Timer.h"
#include <tbb/tbb.h>
#include "tbb/concurrent_vector.h"
#include "tbb/concurrent_unordered_map.h"
using namespace open3d;
using namespace open3d::visualization::gui;
using namespace open3d::visualization::rendering;

namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI UINT RenderToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths);
NCRAFTIMAGEGENAPI UINT RenderToImage(std::filesystem::path& appPath, std::filesystem::path& filePath);
UINT RenderModelToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath);
UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath, std::shared_ptr<geometry::PointCloud> new_cloud_ptr);
UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPointCloudFilesParallel(std::vector<std::filesystem::path>& batchModeFilenames, tbb::concurrent_vector<std::shared_ptr<geometry::PointCloud>>& cloudPtrs);

NCRAFTIMAGEGENAPI extern std::vector<std::string> ModelFileExtensions;
NCRAFTIMAGEGENAPI extern std::vector<std::string> PointcloudFileExtensions;
}

#pragma warning(pop)
