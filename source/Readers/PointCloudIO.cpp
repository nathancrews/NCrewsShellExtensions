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

#include "Renderers/RenderToImageCommon.h"
#include "Readers/PointcloudIO.h"
#include "Tools/MathHelper.h"

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/PointCloudIO.h"
#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/KDTreeSearchParam.h"

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "lasreader.hpp"
#include "laswaveform13reader.hpp"


namespace NCrewsPointCloudTools
{


UINT LoadPointCloudFile(NCrewsImageGen::FileProcessPackage& outLoadResults, pcl::PointXYZRGB* mergedBasePoint)
{
    int pointCount = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;


    if (outLoadResults.m_fileSize < 10)
    {
        return pointCount;
    }

    utility::Timer timer2;
    timer2.Start();

    if (!outLoadResults.m_FileName.extension().compare(".las") ||
        !outLoadResults.m_FileName.extension().compare(".laz") ||
        !outLoadResults.m_FileName.extension().compare(".LAS") ||
        !outLoadResults.m_FileName.extension().compare(".LAZ"))
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();
        if (new_cloud_ptr)
        {
            pointCount = LoadLASorLAZToO3D(outLoadResults.m_FileName, *new_cloud_ptr, mergedBasePoint);
            outLoadResults.m_pointCount = pointCount;
            new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
            outLoadResults.m_cloudPtr = new_cloud_ptr;
        }
    }

    timer2.Stop();
    double exeTimeInner = timer2.GetDurationInSecond();

    if (pointCount > 0)
    {
        outLoadResults.m_processTimeSeconds = exeTimeInner;

        utility::LogInfo("=====>Point cloud {}, points:{}, Load Duration: {} seconds\n",
                         outLoadResults.m_FileName.filename().string(), pointCount, exeTimeInner);
    }

    return pointCount;
}

UINT PackToSingleCloud(tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& filesToLoad,
                       std::shared_ptr<geometry::PointCloud>& cloud_ptr)
{
    UINT retval = 0;
    size_t neighbors = 25;
    double distRatio = 10.0;

    for (NCrewsImageGen::FileProcessPackage& fileToLoad : filesToLoad)
    {
        if (fileToLoad.m_cloudPtr != nullptr && fileToLoad.m_cloudPtr->IsEmpty() == false)
        {
            *cloud_ptr += *fileToLoad.m_cloudPtr;

            retval++;

            fileToLoad.m_cloudPtr->Clear();
        }
    }
    return retval;
}

UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outLoadResults,
                                 pcl::PointXYZRGB* mergedBasePoint)
{
    int pointCountNP = 0;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;

    utility::Timer timer;


    if (outLoadResults.size() == 0)
    {
        return 0;
    }

    timer.Start();

    if (outLoadResults.size() > 0)
    {
        if (outLoadResults[0].m_imageFileCacheOk == false)
        {
            pointCountNP = LoadPointCloudFile(outLoadResults[0], mergedBasePoint);

            pointCountTotal = pointCountNP;
        }

#pragma omp parallel for reduction (+ : pointCountTotal)
        for (int sz = 1; sz < outLoadResults.size(); ++sz)
        {
            int pointCount = 0;

            if (outLoadResults[sz].m_imageFileCacheOk == false)
            {
                utility::Timer timer2;
                timer2.Start();

                pointCount = LoadPointCloudFile(outLoadResults[sz], mergedBasePoint);

                timer2.Stop();
                double exeTimeInner = timer2.GetDurationInSecond();

                if (pointCount > 0)
                {
                    outLoadResults[sz].m_pointCount = pointCount;
                    outLoadResults[sz].m_processTimeSeconds = exeTimeInner;

                    utility::LogInfo("@@===>Threaded Point cloud {}, points:{}, Load Duration: {} seconds\n",
                                     outLoadResults[sz].m_FileName.filename().string(), pointCount, exeTimeInner);

                    pointCountTotal += pointCount;
                }
            }
        }
    }

    return pointCountTotal;
}


UINT LoadLASorLAZToO3D(const std::filesystem::path& fileName, geometry::PointCloud& pointcloud, pcl::PointXYZRGB* pCommonBasePoint)
{
    double px = 0.0, py = 0.0, pz = 0.0;
    double pBasex = 0.0, pBasey = 0.0, pBasez = 0.0;
    uint64_t pointCount = 0;
    // color from point intensity factor
    double cfactor = 255.0 / 65536.0;
    uint64_t pointsInFile = 0;
    pcl::PointXYZRGB pclPoint, point;
    uint64_t pointStep = 1;
    I64 pointToPixel = 750000;
    Eigen::Vector3d o3dPoint, o3dColor;

    LASreadOpener lasreadopener;
    LASreader* lasreader = lasreadopener.open(fileName.string().c_str(), FALSE);

    if (lasreader)
    {
        pointsInFile = lasreader->npoints;

        if (lasreader->npoints > pointToPixel)
        {
            pointStep = lasreader->npoints / pointToPixel;
        }

        if (pointStep <= 0)
        {
            pointStep = 1;
        }

        utility::LogInfo("PointStep = {}, {} Points in file {}", pointStep, pointsInFile, fileName.filename().string());

        bool basePointSet = (!MathHelper::IsFuzzyEqual(pCommonBasePoint->x, 0.0f) && !MathHelper::IsFuzzyEqual(pCommonBasePoint->y, 0.0f));
        switch (basePointSet)
        {
            case false:
                pBasex = lasreader->header.min_x;
                pBasey = lasreader->header.min_y;
                pBasez = lasreader->header.min_z;

                pCommonBasePoint->x = pBasex;
                pCommonBasePoint->y = pBasey;
                pCommonBasePoint->z = pBasez;
                break;

            case true:
                pBasex = pCommonBasePoint->x;
                pBasey = pCommonBasePoint->y;
                pBasez = pCommonBasePoint->z;
                break;
        }

        while (lasreader->read_point())
        {
            if (MathHelper::IsFuzzyZero(std::fmodl(lasreader->p_count, pointStep)))
            {

                o3dPoint = { 0.0f, 0.0f, 0.0f };
                o3dColor = { 0.0f, 0.0f, 0.0f };

                lasreader->point.compute_coordinates();

                o3dPoint[0] = (lasreader->point.coordinates[0] - pBasex);
                o3dPoint[1] = (lasreader->point.coordinates[1] - pBasey);
                o3dPoint[2] = (lasreader->point.coordinates[2] - pBasez);

                ////////////////////////////////////////////////////////////////////////
                // colors
                LASpoint* pntForColor = &lasreader->point;
                LASheader* headerForColor = &lasreader->header;

                if (ComputeLASPointColor(headerForColor, pntForColor, o3dColor))
                {
                    pointcloud.points_.push_back(o3dPoint);
                    pointcloud.colors_.push_back(o3dColor);

                    pointCount++;
                }
                ////////////////////////////////////////////////////////////////////////
            }
        }

        lasreader->close();
    }


    return pointsInFile;
}

bool ComputeLASPointColor(const LASheader* lHeader, const LASpoint* lPnt, Eigen::Vector3d& outPntColor)
{
    if (!lPnt || !lHeader)
    {
        return false;
    }

    outPntColor = PCColor::Black;

    if (lPnt->have_rgb)
    {
        // 16bit RBG bit data
        outPntColor[0] = (double)((lPnt->get_R() >> 8) / 255.f);
        outPntColor[1] = (double)((lPnt->get_G() >> 8) / 255.f);
        outPntColor[2] = (double)((lPnt->get_B() >> 8) / 255.f);

        // fixed up for 8bit RBG bit data
        if (MathHelper::IsFuzzyZero(outPntColor.x()) && MathHelper::IsFuzzyZero(outPntColor.y()))
        {
            outPntColor[0] = (double)(lPnt->get_R() / 255.f);
            outPntColor[1] = (double)(lPnt->get_G() / 255.f);
            outPntColor[2] = (double)(lPnt->get_B() / 255.f);
        }
    }

    if (outPntColor != PCColor::Black)
    {
        return true;
    }

    if (lPnt->get_intensity() > 0)
    {
        double iScale = 1.0f;
        U16 pIntensity = lPnt->get_intensity();

        if (pIntensity > 256)
        {
            iScale = 0.25f;
        }

        outPntColor[0] = (double)((pIntensity * iScale) / 255.0f);
        outPntColor[1] = (double)((pIntensity * iScale) / 255.0f);
        outPntColor[2] = (double)((pIntensity * iScale) / 255.0f);
    }

    if (outPntColor == PCColor::Black)
    {
        double red = 1.0, green = 0.0, blue = 0.0;
        double deltaElev = lHeader->max_z - lHeader->min_z;

        if (deltaElev < 0.001 || deltaElev > 200.0)
        {
            deltaElev = 200.0;
        }

        double normalizedElevation = 1.0 - (lHeader->max_z - lPnt->coordinates[2]) / deltaElev;

        if (normalizedElevation <= 0.5)
        {
            // Interpolate between 0f, 0f, 1f and 0f, 1f, 0f
            red = 0.0;
            green = normalizedElevation / 0.5;
            blue = 1.0 - green;
        }
        else
        {
            // Interpolate between 0f, 1f, 0f and 1f, 0f, 0f
            red = (normalizedElevation - 0.5) / 0.5;
            green = 1.0 - red;
            blue = 0.0;
        }

        outPntColor[0] = red;
        outPntColor[1] = green;
        outPntColor[2] = blue;
    }

    if (outPntColor != PCColor::Black)
    {
        return true;
    }

    if (lPnt->classification > 1)
    {
        switch ((LASpointClassificationEnum)lPnt->classification)
        {
            case LASpointClassificationEnum::unclassified:
                outPntColor = PCColor::Brown;
                break;
            case LASpointClassificationEnum::ground:
                outPntColor = PCColor::Green;
                break;
            case LASpointClassificationEnum::low_vegetation:
                outPntColor = PCColor::GreenMedium;
                break;
            case LASpointClassificationEnum::medium_vegetation:
                outPntColor = PCColor::GreenLight;
                break;
            case LASpointClassificationEnum::high_vegetation:
                outPntColor = PCColor::GreenLight;
                break;
            case LASpointClassificationEnum::building:
                outPntColor = PCColor::Cyan;
                break;
            case LASpointClassificationEnum::water:
                outPntColor = PCColor::Blue;
                break;
            case LASpointClassificationEnum::road_surface:
                outPntColor = PCColor::GrayDark;
                break;
            case LASpointClassificationEnum::bridge_deck:
                outPntColor = PCColor::GrayDark;
                break;
            case LASpointClassificationEnum::noise:
                outPntColor = PCColor::GrayDark;
                break;
            case LASpointClassificationEnum::overlap:
                outPntColor = PCColor::GrayLight;
                break;
            case LASpointClassificationEnum::tower:
                outPntColor = PCColor::Cyan;
                break;
            case LASpointClassificationEnum::wire_conductor:
                outPntColor = PCColor::Magenta;
                break;
            case LASpointClassificationEnum::wire_guard:
                outPntColor = PCColor::Magenta;
                break;
            case LASpointClassificationEnum::wire_connector:
                outPntColor = PCColor::Magenta;
                break;
        }
    }

    return (outPntColor != PCColor::Black);
}


}

#pragma warning(pop)
