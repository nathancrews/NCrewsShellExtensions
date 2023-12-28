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

using namespace open3d;
using namespace open3d::visualization::gui;
using namespace open3d::visualization::rendering;

namespace NCraftImageGen
{

NCRAFTIMAGEGENAPI int RenderToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths);
NCRAFTIMAGEGENAPI int RenderToImage(std::filesystem::path& appPath, std::filesystem::path& filePath);
int RenderModelToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath);
int RenderPointcloudToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath, std::shared_ptr<geometry::PointCloud> new_cloud_ptr);
int LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);

NCRAFTIMAGEGENAPI extern std::vector<std::string> ModelFileExtensions;
NCRAFTIMAGEGENAPI extern std::vector<std::string> PointcloudFileExtensions;
}

#pragma warning(pop)
