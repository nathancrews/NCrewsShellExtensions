#pragma once
#include "framework.h"
#include <filesystem>
#include <objidl.h>
#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

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

struct ImageGenResult
{
    ImageGenResult() {};
    ImageGenResult(std::filesystem::path& filePath) { m_FileName = filePath; };

    UINT m_modelType = 0; // 0 = point cloud, 1 = gltf
    std::filesystem::path  m_FileName;
    std::filesystem::path m_ImageName;
    UINT m_fileSize = 0;
    UINT m_pointCount = 0;
    double m_processTimeSeconds = 0.0;
    std::shared_ptr<geometry::PointCloud> m_cloudPtr;
};


NCRAFTIMAGEGENAPI UINT RenderToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths, 
                                      tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults);
UINT RenderModelToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo);
UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo);
UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outLoadResults);
UINT GetFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::filesystem::path>& outDirectoryFilenames);
NCRAFTIMAGEGENAPI int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);

NCRAFTIMAGEGENAPI extern std::vector<std::string> ModelFileExtensions;
NCRAFTIMAGEGENAPI extern std::vector<std::string> PointcloudFileExtensions;

// NCRAFTIMAGEGENAPI UINT RenderToImage(std::filesystem::path& appPath, std::filesystem::path& filePath);
}

#pragma warning(pop)
