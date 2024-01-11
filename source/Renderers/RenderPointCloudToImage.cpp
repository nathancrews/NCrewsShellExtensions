#include "Renderers/RenderPointcloudToImage.h"

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/PointCloudIO.h"

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/PtsReader.hpp>
#include <pdal/io/PlyReader.hpp>
#include <pdal/io/PcdReader.hpp>
#include <pdal/io/TextReader.hpp>
#include <pdal/io/LasHeader.hpp>

#include "pcl/PCLHeader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>

using namespace pdal;
using namespace pdal::las;

namespace NCraftImageGen
{

UINT RenderPointcloudFiles(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                           NCraftImageGen::ImageGenSettings& imageSettings,
                           tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults)
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

    std::list<NCraftImageGen::ImageGenResult> sortedLoadResults;

    timer.Start();

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        NCraftImageGen::ImageGenResult toAddResult(reqPath);

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

    std::function<bool(NCraftImageGen::ImageGenResult&, NCraftImageGen::ImageGenResult&)> sort
        = [](NCraftImageGen::ImageGenResult& x, NCraftImageGen::ImageGenResult& y)
        {
            return (x.m_fileSize < y.m_fileSize);
        };

    // sort files by file size
    sortedLoadResults.sort(sort);

    outRenderResults.clear();

    for (NCraftImageGen::ImageGenResult res : sortedLoadResults)
    {
        outRenderResults.push_back(res);
    }

    int maxThreads = siSysInfo.dwNumberOfProcessors - (siSysInfo.dwNumberOfProcessors / 4);
    utility::LogInfo("processing {} files, using {} threads....", outRenderResults.size(), maxThreads);

    for (int outerCount = 0; outerCount < outRenderResults.size(); outerCount += maxThreads)
    {
        tbb::concurrent_vector<NCraftImageGen::ImageGenResult> innerRenderResults;

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

        pointCountTotal = LoadPointCloudFilesParallel(innerRenderResults);

        pointsTotal += pointCountTotal;

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

UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenSettings& imageSettings,
                             NCraftImageGen::ImageGenResult& fileInfo)
{
    UINT millionVal = 1000000;
    UINT kVal = 1000;
    char pointCountStr[MAX_PATH] = { 0 };
    char timeStr[MAX_PATH] = { 0 };
    char fileSizeStr[MAX_PATH] = { 0 };
    std::string infoText;

    std::filesystem::path imagePath = fileInfo.m_FileName;

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
            scene->ShowAxes(false);

            scene->AddGeometry(fileInfo.m_FileName.string(), fileInfo.m_cloudPtr.get(), pointcloud_mat, false);
            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                double max_dim = double(0.3 * bounds.GetMaxExtent());

                if (bounds.GetExtent().z() > std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) / 3.0)
                {
                    utility::LogInfo("Using alternate Camera Z...");

                    Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                    Eigen::Vector3f eye, up;

                    double zcamVec = std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) * 0.20;

                    eye = Eigen::Vector3f(center.x() + (max_dim / 1.0),
                                          center.y() + (max_dim / 1.0),
                                          bounds.min_bound_.z() + zcamVec);
                    Eigen::Vector3f newCenter(center.x(), center.y(), bounds.min_bound_.z());
                    up = Eigen::Vector3f(0, 0, 1);

                    scene->GetCamera()->LookAt(newCenter, eye, up);
                }
                else
                {
                    Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                    Eigen::Vector3f eye, up;

                    eye = Eigen::Vector3f(center.x() + (max_dim / 1.0),
                                          center.y() + (max_dim / 1.0),
                                          center.z() + (max_dim / 1.0));
                    up = Eigen::Vector3f(0, 0, 1);

                    Eigen::Vector3f newCenter(center.x(), center.y(), bounds.min_bound_.z());

                    scene->GetCamera()->LookAt(newCenter, eye, up);
                }

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

                    infoText = "Produced by: NCraft Pointcloud Shell Extension 1.0";
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
                    if (imageSettings.is_Licensed == false)
                    {
                        textColor[0] = 0;
                        textColor[1] = 255;
                        textColor[2] = 255;
                        double lineWidth = 25.0;
                        textScale = imageSettings.imageWidth / 75;

                        cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

                        nextTextRow = ts.height + imageSettings.imageHeight / 3.0;

                        infoText = "FREE";

                        cv::putText(imgWithText2, infoText, cv::Point(0, nextTextRow),
                                    cv::FONT_HERSHEY_SIMPLEX, textScale, textColor, lineWidth);
                    }

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

            }

            delete scene;
        }
    }

    return 1;
}

UINT RenderPointcloudFilesToSingleImage(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                                        NCraftImageGen::ImageGenSettings& imageSettings,
                                        tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults)
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

    if (imageSettings.is_Licensed == false)
    {
        if (batchModeFilenames.size() > 3)
        {
            MessageBox(nullptr, L"Free version merge limit is three files.", L"NCraft Cloud Extension", MB_OK);

            return pointsTotal;
        }
    }

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

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        NCraftImageGen::ImageGenResult toAddResult(reqPath);

        uintmax_t fsize = 0;
        __std_win_error wep = std::filesystem::_File_size(reqPath, fsize);

        toAddResult.m_fileSize = fsize;
        toAddResult.m_modelType = 0;

        std::filesystem::path imagePath = reqPath;
        toAddResult.m_ImageName = imagePath.replace_extension(imageSettings.imageFormat);

        outRenderResults.push_back(toAddResult);
    }

    utility::Timer timer2;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;

    timer2.Start();

    pointCountTotal = LoadPointCloudFilesParallel(outRenderResults);

    pointsTotal += pointCountTotal;

    timer2.Stop();
    execExecTotal = timer2.GetDurationInSecond();

    utility::LogInfo("[Single Image Files: {}] ==>Total points {}, Loading Process Duration: {} seconds", outRenderResults.size(), pointCountTotal, execExecTotal);

    FilamentRenderer* renderer =
        new FilamentRenderer(EngineInstance::GetInstance(), imageSettings.imageWidth, imageSettings.imageHeight,
                             EngineInstance::GetResourceManager());

    if (renderer)
    {
        RenderPointcloudsToImage(renderer, imageSettings, outRenderResults);

        for (NCraftImageGen::ImageGenResult res : outRenderResults)
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

UINT RenderPointcloudsToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenSettings& imageSettings,
                              tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& fileInfos)
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
    std::wstring mergedImageName = L"Merged_" + imageName;

    imagePath = imagePath.replace_filename(mergedImageName);
    imagePath = imagePath.replace_extension(imageSettings.imageFormat);

    fileInfos[0].m_ImageName = imagePath;

    for (NCraftImageGen::ImageGenResult fileInfo : fileInfos)
    {
        totalPointCount += fileInfo.m_pointCount;
        totalFileSize += fileInfo.m_fileSize;

        if (fileInfo.m_cloudPtr)
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
            utility::LogInfo("Added point cloud file: {}", fileInfo.m_FileName.string());
        }
    }

    auto& bounds = scene->GetBoundingBox();
    std::shared_ptr<geometry::Image> img;

    if (bounds.GetMaxExtent() > 0.0f)
    {
        float max_dim = float(0.35 * bounds.GetMaxExtent());
        if (bounds.GetExtent().z() > std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) / 3.0)
        {
            utility::LogInfo("Using alternate Camera Z...");

            Eigen::Vector3f center = bounds.GetCenter().cast<float>();
            Eigen::Vector3f eye, up;

            double zcamVec = std::max(bounds.GetExtent().x(), bounds.GetExtent().y()) * 0.20;

            eye = Eigen::Vector3f(center.x() + (max_dim / 1.0),
                                  center.y() + (max_dim / 1.0),
                                  bounds.min_bound_.z() + zcamVec);
            Eigen::Vector3f newCenter(center.x(), center.y(), bounds.min_bound_.z());
            up = Eigen::Vector3f(0, 0, 1);

            scene->GetCamera()->LookAt(newCenter, eye, up);
        }
        else
        {
            Eigen::Vector3f center = bounds.GetCenter().cast<float>();
            Eigen::Vector3f eye, up;

            eye = Eigen::Vector3f(center.x() + (max_dim / 1.0),
                                  center.y() + (max_dim / 1.0),
                                  center.z() + (max_dim / 1.0));
            up = Eigen::Vector3f(0, 0, 1);

            Eigen::Vector3f newCenter(center.x(), center.y(), bounds.min_bound_.z());

            scene->GetCamera()->LookAt(newCenter, eye, up);
        }

        auto callback = [&img](std::shared_ptr<geometry::Image> _img)
            {
                img = _img;
            };

        scene->GetView()->SetViewport(0, 0, imageSettings.imageWidth, imageSettings.imageHeight);
        modelRenderer->RenderToImage(scene->GetView(), scene->GetScene(), callback);
        modelRenderer->BeginFrame();
        modelRenderer->EndFrame();

        scene->ClearGeometry();
        delete scene;
    }

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

        infoText = "Produced by: NCraft Pointcloud Shell Extension 1.0";
        cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

        unsigned int nextTextRow = ts.height + rowSpacing;

        cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                    cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

        textScale = 1.0;

        infoText = "Merged file reference: " + fileInfos[0].m_FileName.filename().string();
        nextTextRow += ts.height + rowSpacing;
        cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                    cv::FONT_HERSHEY_SIMPLEX, textScale * 0.75, textColor);

        if (totalFileSize > 1048576 * 1000)
        {
            sprintf(fileSizeStr, "Source files total size: %0.3f GB", (double)(totalFileSize) / (double)(1048576 * 1000));
        }
        else if (totalFileSize > 1048576)
        {
            sprintf(fileSizeStr, "Source files total size: %0.3f MB", (double)(totalFileSize) / (double)(1048576));
        }
        else
        {
            sprintf(fileSizeStr, "Source files total size: %0.2f KB", (double)(totalFileSize / (double)1048));
        }

        nextTextRow += ts.height + rowSpacing;

        infoText = fileSizeStr;

        cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                    cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

        if (totalPointCount > millionVal)
        {
            sprintf(pointCountStr, "%0.3f M", (double)(totalPointCount) / (double)millionVal);
        }
        else if (totalPointCount > kVal)
        {
            sprintf(pointCountStr, "%0.3f K", (double)(totalPointCount) / (double)kVal);
        }
        else
        {
            sprintf(pointCountStr, "%d", totalPointCount);
        }

        nextTextRow += ts.height + rowSpacing;

        infoText = "Total points rendered : " + std::string(pointCountStr);

        cv::putText(imgWithText2, infoText, cv::Point(horOffset, nextTextRow),
                    cv::FONT_HERSHEY_SIMPLEX, textScale, textColor);

        //*******************************************************************************************************
        // Check license and add watermark
        if (imageSettings.is_Licensed == false)
        {
            textColor[0] = 0;
            textColor[1] = 255;
            textColor[2] = 255;
            double lineWidth = 25.0;
            textScale = imageSettings.imageWidth / 75;

            cv::Size ts = cv::getTextSize(infoText, cv::FONT_HERSHEY_SIMPLEX, textScale, lineWidth, &bl);

            nextTextRow = ts.height + imageSettings.imageHeight / 3.0;

            infoText = "FREE";

            cv::putText(imgWithText2, infoText, cv::Point(0, nextTextRow),
                        cv::FONT_HERSHEY_SIMPLEX, textScale, textColor, lineWidth);
        }

        //*******************************************************************************************************
        utility::LogInfo("Writing image file with text: {}", imagePath.string());
        cv::imwrite(imagePath.string().c_str(), imgWithText2);

        imgWithText2.release();
    }
    else
    {
        utility::LogInfo("Writing image file: {}", imagePath.string());
        io::WriteImage(imagePath.string(), *img);
    }

    return 1;
}

UINT LoadPointCloudFile(NCraftImageGen::ImageGenResult& outLoadResults, pcl::PointXYZRGB* mergedBasePoint)
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

            pointCount = LoadLASorLAZToO3DCloud(outLoadResults.m_FileName, *new_cloud_ptr, mergedBasePoint);
            outLoadResults.m_pointCount = pointCount;
            new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
            outLoadResults.m_cloudPtr = new_cloud_ptr;
        }
    }
    else if (!outLoadResults.m_FileName.extension().compare(".ply"))
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

        pointCount = LoadPLYToO3DCloud(outLoadResults.m_FileName, *new_cloud_ptr);
        outLoadResults.m_pointCount = pointCount;

        if (pointCount > 0)
        {
            new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
            outLoadResults.m_cloudPtr = new_cloud_ptr;
        }

    }
    else if (!outLoadResults.m_FileName.extension().compare(".pts"))
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

        pointCount = LoadPTSToO3DCloud(outLoadResults.m_FileName, *new_cloud_ptr);
        outLoadResults.m_pointCount = pointCount;

        if (pointCount > 0)
        {
            new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
            outLoadResults.m_cloudPtr = new_cloud_ptr;
        }

    }
    else if (!outLoadResults.m_FileName.extension().compare(".xyz"))
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

        pointCount = LoadXYZToO3DCloud(outLoadResults.m_FileName, *new_cloud_ptr);
        outLoadResults.m_pointCount = pointCount;

        if (pointCount > 0)
        {
            new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
            outLoadResults.m_cloudPtr = new_cloud_ptr;
        }

    }
    else if (!outLoadResults.m_FileName.extension().compare(".pcd"))
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

        if (new_cloud_ptr)
        {
            if (io::ReadPointCloud(outLoadResults.m_FileName.string(), *new_cloud_ptr))
            {
                pointCount = new_cloud_ptr->points_.size();
                outLoadResults.m_pointCount = pointCount;

                if (pointCount > 0)
                {
                    new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
                    outLoadResults.m_cloudPtr = new_cloud_ptr;
                }
            }
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


UINT LoadPointCloudFilesParallel(tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outLoadResults)
{
    UINT div = 1048576;
    int pointCount = 0;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;

    utility::Timer timer;
    pcl::PointXYZRGB mergedBasePoint;
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    timer.Start();

    if (outLoadResults.size() == 1)
    {
        if (outLoadResults[0].m_imageFileCacheOk == false)
        {
            pointCount = LoadPointCloudFile(outLoadResults[0], &mergedBasePoint);
            pointCountTotal = pointCount;
        }
    }
    else
    {
        if (outLoadResults[0].m_imageFileCacheOk == false)
        {
            if (outLoadResults[0].m_imageFileCacheOk == false)
            {
                pointCount = LoadPointCloudFile(outLoadResults[0], &mergedBasePoint);
                pointCountTotal = pointCount;
            }
        }

#pragma omp parallel for
        for (int sz = 1; sz < outLoadResults.size(); ++sz)
        {
            if (outLoadResults[sz].m_imageFileCacheOk == false)
            {
                MEMORYSTATUSEX statex;
                statex.dwLength = sizeof(statex);
                GlobalMemoryStatusEx(&statex);

                utility::Timer timer2;
                timer2.Start();

                pointCount = LoadPointCloudFile(outLoadResults[sz], &mergedBasePoint);

                timer2.Stop();
                double exeTimeInner = timer2.GetDurationInSecond();

                if (pointCount > 0)
                {
                    outLoadResults[sz].m_pointCount = pointCount;
                    outLoadResults[sz].m_processTimeSeconds = exeTimeInner;

                    utility::LogInfo("@@===>Threaded Point cloud {}, points:{}, Load Duration: {} seconds\n",
                                     outLoadResults[sz].m_FileName.filename().string(), pointCount, exeTimeInner);

                    omp_set_lock(&writelock);
                    pointCountTotal += pointCount;
                    omp_unset_lock(&writelock);
                }
            }
        }

        omp_destroy_lock(&writelock);
    }

    return pointCountTotal;
}


UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud, pcl::PointXYZRGB* pCommonBasePoint)
{
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

    pdal::BOX3D bnds = lsHeader.getBounds();
    double deltaElev = bnds.maxz - bnds.minz;
    double greyScaleFactor = 0.90;

    if (deltaElev < 0.001 || deltaElev > 200.0)
    {
        deltaElev = 200.0;

        bnds.maxz = bnds.minz + deltaElev;
    }

    utility::LogInfo("LAS...Point cloud min z {}, max z {}, \n", bnds.minz, bnds.maxz);

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



}

#pragma warning(pop)


#if 0

UINT RenderToImage(std::filesystem::path& appPath, std::filesystem::path& filePath)
{
    const int width = 2048;
    const int height = 1640;
    std::filesystem::path resourcePath = appPath;
    resourcePath += "resources";

    EngineInstance::SetResourcePath(resourcePath.string().c_str());

    FilamentRenderer* renderer =
        new FilamentRenderer(EngineInstance::GetInstance(), width, height,
                             EngineInstance::GetResourceManager());

    if (!renderer)
    {
        return 0;
    }

    std::filesystem::path path = filePath;
    std::vector<std::filesystem::path> batchModeFilenames, batchPointcloudFilenames;

    // directory for multiple file requested
    if (std::filesystem::is_directory(path))
    {
        if (std::filesystem::exists(path))
        {
            for (auto const& dir_entry :
                 std::filesystem::recursive_directory_iterator(path))
            {
                if (dir_entry.is_regular_file())
                {
                    for (std::string fext : ModelFileExtensions)
                    {
                        if (!dir_entry.path().extension().compare(fext))
                        {
                            batchModeFilenames.push_back(dir_entry.path());
                            break;
                        }
                    }

                    for (std::string pcext : PointcloudFileExtensions)
                    {
                        if (!dir_entry.path().extension().compare(pcext))
                        {
                            batchModeFilenames.push_back(dir_entry.path());
                            break;
                        }
                    }
                }
            }
        }
    }
    else // single file requested
    {
        for (std::string fext : ModelFileExtensions)
        {
            if (!path.extension().compare(fext))
            {
                batchModeFilenames.push_back(path);
                break;
            }
        }

        for (std::string pcext : PointcloudFileExtensions)
        {
            if (!path.extension().compare(pcext))
            {
                batchModeFilenames.push_back(path);
                break;
            }
        }
    }

    utility::LogInfo("processing {} files....\n", batchModeFilenames.size());

    utility::Timer timer;
    int pointCountTotal = 0;
    double exeTime = 0.0, execExecTotal = 0.0;
    tbb::concurrent_vector<std::shared_ptr<geometry::PointCloud>> cloudPtrs;

    timer.Start();

    if (batchModeFilenames.size() > 2)
    {
        pointCountTotal = LoadPointCloudFilesParallel(batchModeFilenames, cloudPtrs);
    }
    else
    {
        std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();
        if (new_cloud_ptr)
        {
            pointCountTotal = LoadLASorLAZToO3DCloud(batchModeFilenames[0], *new_cloud_ptr);
            if (pointCountTotal > 0)
            {
                new_cloud_ptr->SetName(batchModeFilenames[0].string());
                cloudPtrs.push_back(new_cloud_ptr);
            }
        }
    }

    timer.Stop();
    exeTime = timer.GetDurationInSecond();
    execExecTotal += exeTime;

    if (pointCountTotal > 0)
    {
        int pntsPerSec = (int)(pointCountTotal / execExecTotal);
        utility::LogInfo("Finished Loading {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}\n", pointCountTotal, exeTime, pntsPerSec);
    }

    for (int sz = 0; sz < batchModeFilenames.size(); ++sz)
    {
        if (!batchModeFilenames[sz].extension().compare(".gltf") ||
            !batchModeFilenames[sz].extension().compare(".glb"))
        {
            timer.Start();
            RenderModelToImage(renderer, batchModeFilenames[sz]);
            timer.Stop();

            exeTime = timer.GetDurationInSecond();
            execExecTotal += exeTime;

            utility::LogInfo("Model Load/Render Process Duration: {} seconds\n", exeTime);
        }
    }

    for (std::shared_ptr<geometry::PointCloud> cloudPtr : cloudPtrs)
    {
        timer.Start();

        std::filesystem::path cNamePath = cloudPtr->GetName();
        RenderPointcloudToImage(renderer, cNamePath, cloudPtr);

        cloudPtr->Clear();

        timer.Stop();

        exeTime = timer.GetDurationInSecond();
        execExecTotal += exeTime;

        utility::LogInfo("Pointcloud Render Duration: {} seconds\n", exeTime);
    }

    if (pointCountTotal > 0)
    {
        utility::LogInfo("==>Total Load/Render for {} files, total points: {}, Duration: {} seconds\n", batchModeFilenames.size(), pointCountTotal, execExecTotal);
    }
    else
    {
        utility::LogInfo("==>Total Load/Render for {} files, Duration: {} seconds\n", batchModeFilenames.size(), execExecTotal);
    }


    delete renderer;

    return 1;
}

#endif