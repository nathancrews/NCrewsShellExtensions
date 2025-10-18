////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan C. Crews IV
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

UINT LoadLASorLAZToO3D(const std::filesystem::path& fileName, geometry::PointCloud& pointcloud, 
                       pcl::PointXYZRGB* pCommonBasePoint);

UINT LoadPointCloudFile(NCrewsImageGen::FileProcessPackage& outLoadResults, pcl::PointXYZRGB* pCommonBasePoint);
UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outLoadResults,
                                 pcl::PointXYZRGB* mergedBasePoint);

bool ComputeLASPointColor(const LASheader* lHeader, const LASpoint* lPnt, Eigen::Vector3d& outPntColor);

}

