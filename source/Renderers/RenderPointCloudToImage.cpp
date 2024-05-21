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


#include "Readers/PointcloudIO.h"
#include "open3d/visualization/utility/GLHelper.h"
#include "Renderers/RenderPointcloudToImage.h"
#include "Tools/MathHelper.h"

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <Eigen/StdVector>
#include <Eigen/Geometry>


namespace NCrewsImageGen
{

UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                           NCrewsImageGen::AppSettings& imageSettings,
                           tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults)
{
    UINT pointsTotal = 0;
    std::filesystem::path resourcePath = appPath;
    resourcePath += L"resources";

    EngineInstance::SetResourcePath(resourcePath.string().c_str());

    SYSTEM_INFO siSysInfo;
    GetSystemInfo(&siSysInfo);

    std::vector<std::filesystem::path> batchModeFilenames;

    for (std::filesystem::path testPath : filePaths)
    {
        if (std::filesystem::is_directory(testPath))
        {
            GetFileNamesFromDirectory(testPath, PointcloudFileExtensions, batchModeFilenames);
        }
        else
        {
            for (std::string pcext : PointcloudFileExtensions)
            {
                if (!testPath.extension().compare(pcext))
                {
                    batchModeFilenames.push_back(testPath);
                    break;
                }
            }
        }
    }

    if (batchModeFilenames.empty())
    {
        return pointsTotal;
    }

    if (batchModeFilenames.size() > 5)
    {
        int retval = MessageBox(nullptr, L"    Proceed to Generate Images?", L"Info: Many files selected", MB_YESNO);

        if (retval != 6)
        {
            return pointsTotal;
        }
    }

    utility::Timer timer;
    double exeTime = 0.0;

    std::list<NCrewsImageGen::FileProcessPackage> sortedLoadResults;

    timer.Start();

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        NCrewsImageGen::FileProcessPackage toAddResult(reqPath);

        uintmax_t fsize = 0;
        __std_win_error wep = std::filesystem::_File_size(reqPath, fsize);

        toAddResult.m_fileSize = fsize;
        toAddResult.m_modelType = 0;

        std::filesystem::file_time_type sourceFiletime = std::filesystem::last_write_time(reqPath);
        std::filesystem::path imagePath = reqPath;
        imagePath = imagePath.replace_extension(imageSettings.imageFormat);

        toAddResult.m_ImageName = imagePath;

        if (std::filesystem::exists(imagePath))
        {
            std::filesystem::file_time_type imageFiletime = std::filesystem::last_write_time(imagePath);

            if (sourceFiletime <= imageFiletime)
            {
                toAddResult.m_imageFileCacheOk = true;
            }
        }

        sortedLoadResults.push_back(toAddResult);
    }

    std::function<bool(NCrewsImageGen::FileProcessPackage&, NCrewsImageGen::FileProcessPackage&)> sort
        = [](NCrewsImageGen::FileProcessPackage& x, NCrewsImageGen::FileProcessPackage& y)
        {
            return (x.m_fileSize < y.m_fileSize);
        };

    // sort files by file size
    sortedLoadResults.sort(sort);

    outRenderResults.clear();

    for (NCrewsImageGen::FileProcessPackage res : sortedLoadResults)
    {
        outRenderResults.push_back(res);
    }

    int maxThreads = siSysInfo.dwNumberOfProcessors;

    if (siSysInfo.dwNumberOfProcessors > 1)
    {
        maxThreads = siSysInfo.dwNumberOfProcessors - (siSysInfo.dwNumberOfProcessors / 4);
    }

    utility::LogInfo("processing {} files, using {} threads....", outRenderResults.size(), maxThreads);

    for (int outerCount = 0; outerCount < outRenderResults.size(); outerCount += maxThreads)
    {
        tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> innerRenderResults;

        for (int innerCount = outerCount; innerCount < (outerCount + maxThreads); ++innerCount)
        {
            if (innerCount == outRenderResults.size())
            {
                outerCount = outRenderResults.size();
                break;
            }

            innerRenderResults.push_back(outRenderResults[innerCount]);
        }

        utility::Timer timer2;
        int pointCountTotal = 0;
        double execExecTotal = 0.0;

        timer2.Start();

        pointCountTotal = NCrewsPointCloudTools::LoadPointCloudFilesParallel(innerRenderResults, &imageSettings.mergedBasePoint);

        pointsTotal = pointCountTotal;

        timer2.Stop();
        execExecTotal = timer2.GetDurationInSecond();

        int pntsPerSec = (int)(pointCountTotal / execExecTotal);
        utility::LogInfo("[INNER COUNT {}] ==>Loaded {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}", innerRenderResults.size(), pointCountTotal, execExecTotal, pntsPerSec);

        FilamentRenderer* renderer =
            new FilamentRenderer(EngineInstance::GetInstance(), imageSettings.imageWidth, imageSettings.imageHeight,
                                 EngineInstance::GetResourceManager());

        if (!renderer)
        {
            return pointsTotal;
        }

        for (int sz = 0; sz < innerRenderResults.size(); ++sz)
        {
            if ((innerRenderResults[sz].m_imageFileCacheOk == false) &&
                innerRenderResults[sz].m_cloudPtr.get())
            {
                RenderPointcloudToImage(renderer, imageSettings, innerRenderResults[sz]);
            }

            if (innerRenderResults[sz].m_cloudPtr.get())
            {
                innerRenderResults[sz].m_cloudPtr->Clear();
            }
        }

        delete renderer;
    }

    timer.Stop();

    exeTime = timer.GetDurationInSecond();

    utility::LogInfo("Finished Loading {} files, points {}, Total Loading Process Duration: {} seconds",
                     outRenderResults.size(), pointsTotal, exeTime);

    return pointsTotal;
}

void SetCameraEyeAndTarget(Open3DScene* scene)
{
    auto& bounds = scene->GetBoundingBox();

    utility::LogInfo("Add geometry and update bounding box to {}", bounds.GetPrintInfo().c_str());

    if (bounds.GetMaxExtent() > 0.0)
    {
        double zoom = 0.95;
        double field_of_view = 85.0;
        double view_ratio_ = (zoom * bounds.GetMaxExtent());

        utility::LogInfo("[Before calc] Z_Near: {}, Z_Far: {}", scene->GetCamera()->GetNear(), scene->GetCamera()->GetFar());

        double newFarPlane = visualization::rendering::Camera::CalcFarPlane(*scene->GetCamera(), bounds);
        double newNearPlane = scene->GetCamera()->CalcNearPlane();

        std::array<int, 4> vp = scene->GetView()->GetViewport();
        double aspect = 1.0;
        if (vp[3] != 0)
        {
            aspect = vp[2] / vp[3];
        }

        scene->GetCamera()->SetProjection(field_of_view, aspect,
                                          newNearPlane, newFarPlane,
                                          visualization::rendering::Camera::FovType::Vertical);

        utility::LogInfo("[After calc] Z_Near: {}, Z_Far: {}", scene->GetCamera()->GetNear(), scene->GetCamera()->GetFar());


        if (bounds.GetExtent().z() > std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) / 3.0)
        {
            Eigen::Vector3f center = bounds.GetCenter().cast<float>();
            Eigen::Vector3f eye, up;

            double zcamVec = std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) * 0.20;

            eye = Eigen::Vector3f(center.x() + (view_ratio_ / 1.0),
                                  center.y() + (view_ratio_ / 1.0),
                                  bounds.min_bound_.z() + zcamVec);
            Eigen::Vector3f newCenter(center.x(), center.y(), center.z());
            up = Eigen::Vector3f(0, 0, 1);

            utility::LogInfo("Alternate View: max_dim {}, zcamVec {}", view_ratio_, zcamVec);
            utility::LogInfo("eye: {},{},{}", eye[0], eye[1], eye[2]);
            utility::LogInfo("target: {},{},{}", newCenter[0], newCenter[1], newCenter[2]);

            scene->GetCamera()->LookAt(newCenter, eye, up);
        }
        else
        {
            Eigen::Vector3d center = bounds.GetCenter();
            double eyeHeight = bounds.GetMaxBound().z();
            Eigen::Vector3d eye = center;

            Eigen::Vector3d up = Eigen::Vector3d(0.0, 0.0, 1.0);

            Eigen::Vector3d front = Eigen::Vector3d(0.0, 0.5, 0.5);

            Eigen::Vector3d newCenter(center.x(), center.y(), center.z());

            view_ratio_ = zoom * bounds.GetMaxExtent();
            eyeHeight = view_ratio_ / std::tan(field_of_view * 0.5 / 180.0 * M_PI);
            eye = newCenter + front * eyeHeight;

            utility::LogInfo("standard View: eyeHeight {}, eye: {},{},{}", eyeHeight, eye[0], eye[1], eye[2]);
            utility::LogInfo("target: {},{},{}", newCenter[0], newCenter[1], newCenter[2]);

            scene->GetCamera()->LookAt(newCenter.cast<float>(), eye.cast<float>(), up.cast<float>());
        }
    }
}


UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCrewsImageGen::AppSettings& imageSettings,
                             NCrewsImageGen::FileProcessPackage& fileInfo)
{
    UINT millionVal = 1000000;
    UINT kVal = 1000;
    char pointCountStr[MAX_PATH] = { 0 };
    char timeStr[MAX_PATH] = { 0 };
    char fileSizeStr[MAX_PATH] = { 0 };
    std::string infoText;

    std::filesystem::path imagePath = fileInfo.m_ImageName;

    imagePath = imagePath.replace_extension(imageSettings.imageFormat);

    fileInfo.m_ImageName = imagePath;

    if (fileInfo.m_cloudPtr->HasPoints())
    {
        auto* scene = new Open3DScene(*modelRenderer);
        if (scene && fileInfo.m_cloudPtr)
        {

            auto pointcloud_mat = visualization::rendering::MaterialRecord();
            pointcloud_mat.shader = "defaultUnlit";
            pointcloud_mat.point_size = 3.25f;
            pointcloud_mat.base_color = { 0.55f, 0.55f, 0.55f, 1.0f };
            pointcloud_mat.sRGB_color = false;
            scene->SetLighting(Open3DScene::LightingProfile::NO_SHADOWS, { 0.5f, -0.5f, -0.5f });
            scene->GetScene()->EnableSunLight(true);
            scene->GetScene()->SetSunLightIntensity(35000);
            Eigen::Vector4f color = { 1.0, 1.0, 1.0, 1.0 };
            scene->SetBackground(color);
            scene->ShowAxes(true);

            scene->AddGeometry(fileInfo.m_FileName.string(), fileInfo.m_cloudPtr.get(), pointcloud_mat, false);

            SetCameraEyeAndTarget(scene);

            std::shared_ptr<geometry::Image> img;
            auto callback = [&img](std::shared_ptr<geometry::Image> _img)
                {
                    img = _img;
                };

            scene->GetView()->SetViewport(0, 0, imageSettings.imageWidth, imageSettings.imageHeight);
            modelRenderer->RenderToImage(scene->GetView(), scene->GetScene(), callback);
            modelRenderer->BeginFrame();
            modelRenderer->EndFrame();

            io::WriteImage(imagePath.string(), *img);

            utility::LogInfo("Creating OpenCV image...");

            cv::Mat imgWithText2;
            cv::Scalar textColor(255, 0, 0);
            double textScale = 1.25;
            double lineWidth = 1.0;
            unsigned int rowSpacing = 25, horOffset = 25;

            imgWithText2.create(imageSettings.imageHeight, imageSettings.imageWidth, CV_8UC3);

            if (imgWithText2.data)
            {
                memcpy(imgWithText2.data, img->data_.data(), img->data_.size());

                utility::LogInfo("Writing text to OpenCV image..");

                int bl = 0;

                infoText = "Produced by: NCrews Pointcloud Shell Extension 1.0";
                cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

                unsigned int nextTextRow = ts.height + rowSpacing;

                cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                            cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

                textScale = 1.0;

                infoText = "Source file name: " + fileInfo.m_FileName.filename().string();
                nextTextRow += ts.height + rowSpacing;
                cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                            cv::FONT_HERSHEY_SIMPLEX, textScale * 0.75, textColor);

                if (fileInfo.m_fileSize > 1048576 * 1000)
                {
                    sprintf(fileSizeStr, "Source file size: %0.3f GB", (double)(fileInfo.m_fileSize) / (double)(1048576 * 1000));
                }
                else if (fileInfo.m_fileSize > 1048576)
                {
                    sprintf(fileSizeStr, "Source file size: %0.3f MB", (double)(fileInfo.m_fileSize) / (double)(1048576));
                }
                else
                {
                    sprintf(fileSizeStr, "Source file size: %0.2f KB", (double)(fileInfo.m_fileSize / (double)1048));
                }

                nextTextRow += ts.height + rowSpacing;

                infoText = fileSizeStr;

                cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                            cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

                if (fileInfo.m_pointCount > millionVal)
                {
                    sprintf(pointCountStr, "%0.3f M", (double)(fileInfo.m_pointCount) / (double)millionVal);
                }
                else if (fileInfo.m_pointCount > kVal)
                {
                    sprintf(pointCountStr, "%0.3f K", (double)(fileInfo.m_pointCount) / (double)kVal);
                }
                else
                {
                    sprintf(pointCountStr, "%zd", fileInfo.m_pointCount);
                }

                nextTextRow += ts.height + rowSpacing;

                utility::LogInfo("text scale {}, origin {}", textScale, nextTextRow);

                infoText = "Number of points : " + std::string(pointCountStr);

                cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                            cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

                //*******************************************************************************************************
                // Check license and add watermark
                //if (imageSettings.is_Licensed == false)
                //{
                //    textColor[0] = 0;
                //    textColor[1] = 255;
                //    textColor[2] = 255;
                //    double lineWidth = 25.0;
                //    textScale = imageSettings.imageWidth / 75;

                //    cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

                //    nextTextRow = ts.height + imageSettings.imageHeight / 3.0;

                //    infoText = "FREE";

                //    cv::putText(imgWithText2, infoText, cv::Point(0, nextTextRow),
                //                cv::FONT_HERSHEY_SIMPLEX, textScale, textColor, lineWidth);
                //}

                //*******************************************************************************************************
                utility::LogInfo("Writing image file with text: {}", fileInfo.m_ImageName.string());
                cv::imwrite(fileInfo.m_ImageName.string().c_str(), imgWithText2);

                imgWithText2.release();
            }
            else
            {
                utility::LogInfo("Writing image file: {}", imagePath.string());
                io::WriteImage(imagePath.string(), *img);
            }

            delete scene;
        }

    }

    return 1;
}

UINT RenderPointcloudFilesToSingleImage(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                        NCrewsImageGen::AppSettings& imageSettings,
                                        tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults)
{

    UINT pointsTotal = 0;
    std::filesystem::path resourcePath = appPath;
    resourcePath += L"resources";

    EngineInstance::SetResourcePath(resourcePath.string().c_str());

    std::vector<std::filesystem::path> batchModeFilenames;

    for (std::filesystem::path testPath : filePaths)
    {
        if (std::filesystem::is_directory(testPath))
        {
            GetFileNamesFromDirectory(testPath, PointcloudFileExtensions, batchModeFilenames);
        }
        else
        {
            for (std::string pcext : PointcloudFileExtensions)
            {
                if (!testPath.extension().compare(pcext))
                {
                    batchModeFilenames.push_back(testPath);
                    break;
                }
            }
        }
    }

    if (batchModeFilenames.empty())
    {
        return pointsTotal;
    }

    //if (imageSettings.is_Licensed == false)
    //{
    //    if (batchModeFilenames.size() > 3)
    //    {
    //        MessageBox(nullptr, L"Free version file merge limit is three files.", L"Point Cloud Shell Extension", MB_OK);

    //        return pointsTotal;
    //    }
    //}

    if (batchModeFilenames.size() > 5)
    {
        int retval = MessageBox(nullptr, L"  Proceed to Generate Merged Image?", L"Info: Many files selected", MB_YESNO);

        if (retval != 6)
        {
            return pointsTotal;
        }
    }

    utility::Timer timer;
    double exeTime = 0.0;

    timer.Start();

    double totalFileSize = 0.0;

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        NCrewsImageGen::FileProcessPackage toAddResult(reqPath);

        toAddResult.m_modelType = 0;

        uintmax_t fsize = 0;
        __std_win_error wep = std::filesystem::_File_size(reqPath, fsize);

        toAddResult.m_fileSize = fsize;

        totalFileSize += toAddResult.m_fileSize;

        std::filesystem::path imagePath = reqPath;
        toAddResult.m_ImageName = imagePath.replace_extension(imageSettings.imageFormat);

        outRenderResults.push_back(toAddResult);
    }

    utility::Timer timer2;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;

    timer2.Start();

    pointCountTotal = NCrewsPointCloudTools::LoadPointCloudFilesParallel(outRenderResults, &imageSettings.mergedBasePoint);

    pointsTotal += pointCountTotal;

    timer2.Stop();
    execExecTotal = timer2.GetDurationInSecond();

    // set the merged file stats on the first file package for reporting
    outRenderResults[0].m_fileSize = totalFileSize;
    outRenderResults[0].m_pointCount = pointCountTotal;
    //std::wstring mergedFilename = outRenderResults[0].m_FileName;
    //outRenderResults[0].m_FileName = L"Merged_Files_" + mergedFilename;

    utility::LogInfo("[Single Image Files: {}] ==>Total points {}, Loading Process Duration: {} seconds", outRenderResults.size(), pointCountTotal, execExecTotal);

    FilamentRenderer* renderer =
        new FilamentRenderer(EngineInstance::GetInstance(), imageSettings.imageWidth, imageSettings.imageHeight,
                             EngineInstance::GetResourceManager());

    if (renderer)
    {
        RenderPointcloudsToImage(renderer, imageSettings, outRenderResults);

        for (NCrewsImageGen::FileProcessPackage res : outRenderResults)
        {
            if (res.m_cloudPtr)
            {
                res.m_cloudPtr->Clear();
            }
        }

        delete renderer;
    }

    timer.Stop();

    exeTime = timer.GetDurationInSecond();

    utility::LogInfo("Single Image: finished Loading {} files, points {}, Total Loading Process Duration: {} seconds",
                     outRenderResults.size(), pointsTotal, exeTime);

    return pointsTotal;
}


UINT RenderPointcloudsToImage(FilamentRenderer* modelRenderer, NCrewsImageGen::AppSettings& imageSettings,
                              tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& fileInfos)
{
    if (fileInfos.size() < 1)
    {
        return 0;
    }

    UINT millionVal = 1000000;
    UINT kVal = 1000;
    UINT totalPointCount = 0;
    UINT totalFileSize = 0;
    char pointCountStr[MAX_PATH] = { 0 };
    char fileSizeStr[MAX_PATH] = { 0 };
    std::string infoText;
    auto* scene = new Open3DScene(*modelRenderer);

    std::filesystem::path imagePath = fileInfos[0].m_FileName;
    std::wstring imageName = fileInfos[0].m_FileName.filename();
    std::wstring mergedImageName = L"Merged_Files_" + imageName;

    imagePath = imagePath.replace_filename(mergedImageName);
    imagePath = imagePath.replace_extension(imageSettings.imageFormat);


    fileInfos[0].m_ImageName = imagePath;

    std::shared_ptr<geometry::PointCloud> scene_cloud_ptr = std::make_shared<geometry::PointCloud>();

    if (!NCrewsPointCloudTools::PackToSingleCloud(fileInfos, scene_cloud_ptr))
    {
        utility::LogInfo("No files specified!...");
        return 0;
    }

    utility::LogInfo("merged files rendering points = {}", scene_cloud_ptr->points_.size());

    for (NCrewsImageGen::FileProcessPackage fileInfo : fileInfos)
    {
        totalPointCount += fileInfo.m_pointCount;
        totalFileSize += fileInfo.m_fileSize;
    }

    fileInfos[0].m_cloudPtr = scene_cloud_ptr;


    RenderPointcloudToImage(modelRenderer, imageSettings, fileInfos[0]);


    //auto pointcloud_mat = visualization::rendering::MaterialRecord();
    //pointcloud_mat.shader = "defaultUnlit";
    //pointcloud_mat.point_size = 3.25f;
    //pointcloud_mat.base_color = { 0.55f, 0.55f, 0.55f, 1.0f };
    //pointcloud_mat.sRGB_color = false;
    //scene->SetLighting(Open3DScene::LightingProfile::NO_SHADOWS, { 0.5f, -0.5f, -0.5f });
    //scene->GetScene()->EnableSunLight(true);
    //scene->GetScene()->SetSunLightIntensity(35000);
    //Eigen::Vector4f color = { 1.0, 1.0, 1.0, 1.0 };
    //scene->SetBackground(color);
    //scene->ShowAxes(true);

    //scene->AddGeometry("Merged Files", scene_cloud_ptr.get(), pointcloud_mat, false);

    //SetCameraEyeAndTarget(scene);

    //std::shared_ptr<geometry::Image> img;

    //auto callback = [&img](std::shared_ptr<geometry::Image> _img)
    //    {
    //        img = _img;
    //    };

    //scene->GetView()->SetViewport(0, 0, imageSettings.imageWidth, imageSettings.imageHeight);
    //modelRenderer->RenderToImage(scene->GetView(), scene->GetScene(), callback);
    //modelRenderer->BeginFrame();
    //modelRenderer->EndFrame();

    //scene->ClearGeometry();
    //delete scene;

    //io::WriteImage(imagePath.string(), *img);

    //utility::LogInfo("Creating OpenCV image...");

    //cv::Mat imgWithText2;
    //cv::Scalar textColor(255, 0, 0);
    //double textScale = 1.25;
    //double lineWidth = 1.0;
    //unsigned int rowSpacing = 25, horOffset = 25;

    //imgWithText2.create(imageSettings.imageHeight, imageSettings.imageWidth, CV_8UC3);

    //if (imgWithText2.data)
    //{
    //    memcpy(imgWithText2.data, img->data_.data(), img->data_.size());

    //    utility::LogInfo("Writing text to OpenCV image..");

    //    int bl = 0;

    //    infoText = "Produced by: NCrews Pointcloud Shell Extension 1.0";
    //    cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

    //    unsigned int nextTextRow = ts.height + rowSpacing;

    //    cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
    //                cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

    //    textScale = 1.0;

    //    infoText = "Merged file reference: " + fileInfos[0].m_FileName.filename().string();
    //    nextTextRow += ts.height + rowSpacing;
    //    cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
    //                cv::FONT_HERSHEY_SIMPLEX, textScale * 0.75, textColor);

    //    if (totalFileSize > 1048576 * 1000)
    //    {
    //        sprintf(fileSizeStr, "Source files total size: %0.3f GB", (double)(totalFileSize) / (double)(1048576 * 1000));
    //    }
    //    else if (totalFileSize > 1048576)
    //    {
    //        sprintf(fileSizeStr, "Source files total size: %0.3f MB", (double)(totalFileSize) / (double)(1048576));
    //    }
    //    else
    //    {
    //        sprintf(fileSizeStr, "Source files total size: %0.2f KB", (double)(totalFileSize / (double)1048));
    //    }

    //    nextTextRow += ts.height + rowSpacing;

    //    infoText = fileSizeStr;

    //    cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
    //                cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

    //    if (totalPointCount > millionVal)
    //    {
    //        sprintf(pointCountStr, "%0.3f M", (double)(totalPointCount) / (double)millionVal);
    //    }
    //    else if (totalPointCount > kVal)
    //    {
    //        sprintf(pointCountStr, "%0.3f K", (double)(totalPointCount) / (double)kVal);
    //    }
    //    else
    //    {
    //        sprintf(pointCountStr, "%d", totalPointCount);
    //    }

    //    nextTextRow += ts.height + rowSpacing;

    //    infoText = "Total points rendered : " + std::string(pointCountStr);

    //    cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
    //                cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

    //    //*******************************************************************************************************
    //    // Check license and add watermark
    //    //if (imageSettings.is_Licensed == false)
    //    //{
    //    //    textColor[0] = 0;
    //    //    textColor[1] = 255;
    //    //    textColor[2] = 255;
    //    //    double lineWidth = 25.0;
    //    //    textScale = imageSettings.imageWidth / 75;

    //    //    cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

    //    //    nextTextRow = ts.height + imageSettings.imageHeight / 3.0;

    //    //    infoText = "FREE";

    //    //    cv::putText(imgWithText2, infoText, cv::Point(0, nextTextRow),
    //    //                cv::FONT_HERSHEY_SIMPLEX, textScale, textColor, lineWidth);
    //    //}

    //    //*******************************************************************************************************
    //    utility::LogInfo("Writing image file with text: {}", imagePath.string());
    //    cv::imwrite(imagePath.string().c_str(), imgWithText2);

    //    imgWithText2.release();
    //}
    //else
    //{
    //    utility::LogInfo("Writing image file: {}", imagePath.string());
    //    io::WriteImage(imagePath.string(), *img);
    //}

    return 1;
}
/* Original pdal library code
UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud, pcl::PointXYZRGB* pCommonBasePoint)
{
    double px = 0.0, py = 0.0, pz = 0.0;
    double pBasex = 0.0, pBasey = 0.0, pBasez = 0.0;
    int pointCount = 0;
    // color from point intensity factor
    double cfactor = 255.0 / 65536.0;
    uint64_t pointsInFile = 0;
    pcl::PointXYZRGB pclPoint;
    UINT pointStep = 1;
    UINT pointToPixel = 750000;
    PointTable table;
    Eigen::Vector3d o3dPoint, o3dColor;
    Options ops1;
    LasReader reader;

    ops1.add("filename", fileName.string());
    reader.setOptions(ops1);

    const LasHeader lsHeader = reader.header();

    reader.prepare(table);
    PointViewSet point_views = reader.execute(table);

    pointsInFile = lsHeader.pointCount();

    utility::LogInfo("LAS/LAZ::::Point cloud file: {}, points:{}", fileName.filename().string(), pointsInFile);

    if (lsHeader.pointCount() < 5)
    {
        return 0;
    }

    if (lsHeader.pointCount() > pointToPixel)
    {
        pointStep = lsHeader.pointCount() / pointToPixel;
    }

    if (pointStep == 0)
    {
        pointStep = 1;
    }

    pdal::BOX3D bnds = lsHeader.getBounds();
    double deltaElev = bnds.maxz - bnds.minz;
    double greyScaleFactor = 0.90;

    if (deltaElev < 0.001 || deltaElev > 200.0)
    {
        deltaElev = 200.0;

        bnds.maxz = bnds.minz + deltaElev;
    }

    utility::LogInfo("LAS...Point cloud min z = {}, max z = {}, views = {}, \n", bnds.minz, bnds.maxz, point_views.size());

    for (PointViewPtr point_view : point_views)
    {
        for (pdal::PointId idx = 0; idx < (point_view->size() - pointStep); idx += pointStep)
        {
            if (pCommonBasePoint && (idx == 0))
            {
                if (pCommonBasePoint->x == 0.0 && pCommonBasePoint->y == 0.0)
                {
                    pBasex = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
                    pBasey = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
                    pBasez = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

                    pCommonBasePoint->x = pBasex;
                    pCommonBasePoint->y = pBasey;
                    pCommonBasePoint->z = pBasez;
                    //utility::LogInfo("LAS/LAZ::::Setting common base point {}, {}, {}", pBasex, pBasey, pBasez);
                }
                else
                {
                    pBasex = pCommonBasePoint->x;
                    pBasey = pCommonBasePoint->y;
                    pBasez = pCommonBasePoint->z;
                    //utility::LogInfo("LAS/LAZ::::Using common base point {}, {}, {}", pBasex, pBasey, pBasez);
                }
            }

            px = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
            py = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
            pz = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

            o3dPoint[0] = px - pBasex;
            o3dPoint[1] = py - pBasey;
            o3dPoint[2] = pz - pBasez;

            pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx) >> 8;
            pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx) >> 8;
            pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx) >> 8;

            if ((pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0))
            {
                pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx);
                pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx);
                pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx);
            }

            // if no color use point intensity
            if (pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0)
            {
                double testVal = point_view->getFieldAs<int>(pdal::Dimension::Id::Intensity, idx);

                //if (testVal < 0.001)
                //{
                double normalizedElevation = 1.0 - (bnds.maxz - pz) / (bnds.maxz - bnds.minz);
                double red = 1.0, green = 0.0, blue = 0.0;

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

//                    utility::LogInfo("Normalized Elev: {}, rgb ={},{},{}", normalizedElevation, red, green, blue);

                o3dColor[0] = red;
                o3dColor[1] = green;
                o3dColor[2] = blue;

                //o3dColor[0] = normalizedElevation * greyScaleFactor;
                //o3dColor[1] = normalizedElevation * greyScaleFactor;
                //o3dColor[2] = normalizedElevation * greyScaleFactor;
            //}
            //else
            //{
                //o3dColor[0] = (double)testVal * cfactor;
                //o3dColor[1] = (double)testVal * cfactor;
                //o3dColor[2] = (double)testVal * cfactor;
            //}
            }
            else
            {
                o3dColor[0] = (double)(pclPoint.r / 255.f);
                o3dColor[1] = (double)(pclPoint.g / 255.f);
                o3dColor[2] = (double)(pclPoint.b / 255.f);

            }

            pointcloud.points_.push_back(o3dPoint);
            pointcloud.colors_.push_back(o3dColor);

            pointCount++;
        }
    }

    return pointsInFile;
}
*/

#if 0

UINT LoadPLYToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud)
{
    bool baseSet = false;
    double px = 0.0, py = 0.0, pz = 0.0;
    double pBasex = 0.0, pBasey = 0.0, pBasez = 0.0;
    int pointCount = 0;
    // color from point intensity factor
    double cfactor = 255.0 / 65536.0;
    uint64_t pointsInFile = 0;
    pcl::PointXYZRGB pclPoint, point;
    int pointStep = 1;
    int pointToPixel = 750000;
    PointTable table;
    Eigen::Vector3d o3dPoint, o3dColor;
    Options ops1;
    PlyReader reader;

    ops1.add("filename", fileName.string());
    reader.setOptions(ops1);

    QuickInfo pInfo = reader.preview();

    if (pInfo.m_pointCount < 1)
    {
        utility::LogInfo("Error 1: PLY... NO points {} \n", pointsInFile);
        return 0;
    }

    pointsInFile = pInfo.m_pointCount;

    utility::LogInfo("::::Point cloud file: {}, points:{}", fileName.filename().string(), pointsInFile);

    if (pointsInFile > pointToPixel)
    {
        pointStep = pointsInFile / pointToPixel;
    }

    utility::LogInfo("PLY...Point cloud points {}, thinning factor: {}, \n", pointsInFile, pointStep);

    reader.prepare(table);
    PointViewSet point_views = reader.execute(table);

    if (point_views.size() < 1)
    {
        utility::LogInfo("Error 2: PLY... NO points {} \n", pointsInFile);
        return 0;
    }

    for (PointViewPtr point_view : point_views)
    {
        for (pdal::PointId idx = 0; idx < (point_view->size() - pointStep); idx += pointStep)
        {
            if (!baseSet && (idx == 0))
            {
                pBasex = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
                pBasey = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
                pBasez = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

                baseSet = true;
            }

            px = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
            py = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
            pz = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

            o3dPoint[0] = px - pBasex;
            o3dPoint[1] = py - pBasey;
            o3dPoint[2] = pz - pBasez;

            pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx) >> 8;
            pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx) >> 8;
            pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx) >> 8;

            if ((pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0))
            {
                pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx);
                pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx);
                pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx);
            }

            // if no color use point intensity
            if (pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0)
            {
                double testVal = point_view->getFieldAs<int>(pdal::Dimension::Id::Intensity, idx);

                o3dColor[0] = (double)testVal * cfactor;
                o3dColor[1] = (double)testVal * cfactor;
                o3dColor[2] = (double)testVal * cfactor;
            }
            else
            {
                o3dColor[0] = (double)(pclPoint.r / 255.f);
                o3dColor[1] = (double)(pclPoint.g / 255.f);
                o3dColor[2] = (double)(pclPoint.b / 255.f);
            }

            pointcloud.points_.push_back(o3dPoint);
            pointcloud.colors_.push_back(o3dColor);

            pointCount++;
        }
    }

    return pointsInFile;
}

UINT LoadPTSToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud)
{
    bool baseSet = false;
    double px = 0.0, py = 0.0, pz = 0.0;
    double pBasex = 0.0, pBasey = 0.0, pBasez = 0.0;
    int pointCount = 0;
    // color from point intensity factor
    double cfactor = 255.0 / 65536.0;
    uint64_t pointsInFile = 0;
    pcl::PointXYZRGB pclPoint, point;
    int pointStep = 1;
    int pointToPixel = 750000;
    PointTable table;
    Eigen::Vector3d o3dPoint, o3dColor;
    Options ops1;
    PtsReader reader;

    ops1.add("filename", fileName.string());
    reader.setOptions(ops1);

    reader.prepare(table);
    PointViewSet point_views = reader.execute(table);

    if (point_views.size() < 1)
    {
        utility::LogInfo("Error 2: PTS... NO points {} \n", pointsInFile);
        return 0;
    }

    pointsInFile = point_views.begin()->get()->size();

    utility::LogInfo("PTS::::Point cloud file: {}, points:{}", fileName.filename().string(), pointsInFile);

    if (pointsInFile > pointToPixel)
    {
        pointStep = pointsInFile / pointToPixel;
    }

    utility::LogInfo("PTS...Point cloud points {}, thinning factor: {}, \n", pointsInFile, pointStep);

    for (PointViewPtr point_view : point_views)
    {
        for (pdal::PointId idx = 0; idx < (point_view->size() - pointStep); idx += pointStep)
        {
            if (!baseSet && (idx == 0))
            {
                pBasex = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
                pBasey = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
                pBasez = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

                baseSet = true;
            }

            px = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
            py = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
            pz = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

            o3dPoint[0] = px - pBasex;
            o3dPoint[1] = py - pBasey;
            o3dPoint[2] = pz - pBasez;

            pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx) >> 8;
            pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx) >> 8;
            pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx) >> 8;

            if ((pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0))
            {
                pclPoint.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx);
                pclPoint.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx);
                pclPoint.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx);
            }

            // if no color use point intensity
            if (pclPoint.r == 0 && pclPoint.g == 0 && pclPoint.b == 0)
            {
                double testVal = point_view->getFieldAs<int>(pdal::Dimension::Id::Intensity, idx);

                o3dColor[0] = (double)testVal * cfactor;
                o3dColor[1] = (double)testVal * cfactor;
                o3dColor[2] = (double)testVal * cfactor;
            }
            else
            {
                o3dColor[0] = (double)(pclPoint.r / 255.f);
                o3dColor[1] = (double)(pclPoint.g / 255.f);
                o3dColor[2] = (double)(pclPoint.b / 255.f);
            }

            pointcloud.points_.push_back(o3dPoint);
            pointcloud.colors_.push_back(o3dColor);

            pointCount++;
        }
    }

    return pointsInFile;
}

UINT LoadXYZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud)
{
    return 0;
}

#endif


}// end namespace


#pragma warning(pop)
