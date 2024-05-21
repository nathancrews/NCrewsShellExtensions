////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan Crews, NCrews Software
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////////


#pragma once
#include "framework.h"
#include <iostream>
#include <memory>
#include <thread>
#include <filesystem>

#ifdef NCRAFTIMAGEGENAPI_EXPORT
#define NCRAFTIMAGEGENAPI __declspec(dllexport)
#elif defined(NCRAFTIMAGEGENAPI_IMPORT)
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

namespace pcl
{
struct PointXYZRGB
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    std::uint8_t r = 0, g = 0, b = 0;
};

}

namespace NCrewsImageGen
{

const struct PCColor
{
    static inline Eigen::Vector3d Black = { 0.0f, 0.0f, 0.0f };
    static inline Eigen::Vector3d GrayLight = { 0.25f, 0.25f, 0.25f };
    static inline Eigen::Vector3d Gray = { 0.50f, 0.50f, 0.50f };
    static inline Eigen::Vector3d GrayDark = { 0.750f, 0.750f, 0.750f };
    static inline Eigen::Vector3d White = { 1.0f, 1.0f, 1.0f };
    static inline Eigen::Vector3d Red = { 1.0f, 0.0f, 0.0f };
    static inline Eigen::Vector3d GreenLight = { 0.0f, 0.5f, 0.0f };
    static inline Eigen::Vector3d GreenMedium = { 0.0f, 0.5f, 0.0f };
    static inline Eigen::Vector3d Green = { 0.0f, 1.0f, 0.0f };
    static inline Eigen::Vector3d Brown = { 0.58f, 0.3f, 0.0f };
    static inline Eigen::Vector3d BrownDark = { 0.58f, 0.3f, 0.0f };
    static inline Eigen::Vector3d Blue = { 0.0f, 0.0f, 1.0f };
    static inline Eigen::Vector3d Yellow = { 1.0f, 1.0f, 0.0f };
    static inline Eigen::Vector3d Magenta = { 1.0f, 0.0f, 1.0f };
    static inline Eigen::Vector3d Cyan = { 0.0f, 0.5f, 0.5f };
};


struct AppSettings
{
    pcl::PointXYZRGB mergedBasePoint = { 0.0f,0.0f,0.0f };
    UINT imageWidth = 1440;
    UINT imageHeight = 1024;
    std::string imageFormat = "png";
    std::string licenseKey;
    UINT is_Licensed = false;
};

struct FileProcessPackage
{
    FileProcessPackage() {};
    FileProcessPackage(std::filesystem::path& filePath) { m_FileName = filePath; };

    bool m_imageFileCacheOk = false;
    UINT m_modelType = 0; // 0 = point cloud, 1 = gltf
    std::filesystem::path  m_FileName;
    std::filesystem::path m_ImageName;
    uintmax_t m_fileSize = 0;
    double m_processTimeSeconds = 0.0;

    uintmax_t m_pointCount = 0;
    std::shared_ptr<geometry::PointCloud> m_cloudPtr;
};

NCRAFTIMAGEGENAPI bool ReadImageGenSettings(std::filesystem::path& appDataPath, AppSettings& outSettings);

NCRAFTIMAGEGENAPI UINT GetFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::string>& allowedFileExtensions, 
                                                 std::vector<std::filesystem::path>& outDirectoryFilenames);
NCRAFTIMAGEGENAPI UINT GetCloudFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::string>& allowedFileExtensions,
                                                      std::vector<std::filesystem::path>& outDirectoryFilenames);

NCRAFTIMAGEGENAPI extern std::vector<std::string> ModelFileExtensions;
NCRAFTIMAGEGENAPI extern std::vector<std::string> PointcloudFileExtensions;

}

#pragma warning(pop)
