#pragma once
#include "Renderers/RenderToImageCommon.h"
#include "lasreader.hpp"
#include "laswaveform13reader.hpp"

namespace NCrewsPointCloudTools
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

const enum class LASpointClassificationEnum : unsigned short
{
  never_classified=0,
  unclassified,
  ground,
  low_vegetation,
  medium_vegetation,
  high_vegetation,
  building,
  noise,
  keypoint,
  water,
  rail,
  road_surface,
  overlap,
  wire_guard,
  wire_conductor,
  tower,
  wire_connector,
  bridge_deck,
  Reserved
};

UINT PackToSingleCloud(tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& filesToLoad,
                       std::shared_ptr<geometry::PointCloud>& cloud_ptr);

UINT LoadLASorLAZToO3D(std::filesystem::path& fileName, geometry::PointCloud& pointcloud, 
                       pcl::PointXYZRGB* pCommonBasePoint);

//UINT LoadPLYToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
//UINT LoadPTSToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);
//UINT LoadXYZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud);

UINT LoadPointCloudFile(NCrewsImageGen::FileProcessPackage& outLoadResults, pcl::PointXYZRGB* pCommonBasePoint);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outLoadResults,
                                 pcl::PointXYZRGB* mergedBasePoint);

bool ComputeLASPointColor(LASheader* lHeader, LASpoint* lPnt, Eigen::Vector3d& outPntColor);

}

