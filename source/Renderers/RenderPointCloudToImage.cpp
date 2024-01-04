#include "Renderers/RenderPointcloudToImage.h"

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/PointCloudIO.h"

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
                           tbb::concurrent_vector<NCraftImageGen::ImageGenResult>& outRenderResults)
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

    if (batchModeFilenames.size() > 5)
    {
        int retval = MessageBox(nullptr, L"    Proceed to Generate Images?", L"Info: Many files selected", MB_YESNO);

        if (retval != 6)
        {
            return 0;
        }
    }

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        NCraftImageGen::ImageGenResult toAddResult(reqPath);

        uintmax_t fsize = 0;
        __std_win_error wep = std::filesystem::_File_size(reqPath, fsize);

        toAddResult.m_fileSize = fsize;
        toAddResult.m_modelType = 0;

        std::filesystem::file_time_type sourceFiletime = std::filesystem::last_write_time(reqPath);
        std::filesystem::path imagePath = reqPath;
        imagePath = imagePath.replace_extension("jpg");

        toAddResult.m_ImageName = imagePath;

        if (std::filesystem::exists(imagePath))
        {
            std::filesystem::file_time_type imageFiletime = std::filesystem::last_write_time(imagePath);

            if (sourceFiletime <= imageFiletime)
            {
                utility::LogInfo("Skipping image, cache file up to date: {}", imagePath.string().c_str());
                toAddResult.m_imageFileCacheOk = true;
            }
        }

        outRenderResults.push_back(toAddResult);
    }

    utility::LogInfo("processing {} files....\n", outRenderResults.size());

    utility::Timer timer;
    int pointCountTotal = 0;
    double exeTime = 0.0, execExecTotal = 0.0;

    tbb::concurrent_vector<std::shared_ptr<geometry::PointCloud>> cloudPtrs;

    timer.Start();

    pointCountTotal = LoadPointCloudFilesParallel(outRenderResults);

    timer.Stop();
    exeTime = timer.GetDurationInSecond();
    execExecTotal += exeTime;

    if (pointCountTotal > 0)
    {
        int pntsPerSec = (int)(pointCountTotal / exeTime);
        utility::LogInfo("==>Loaded {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}\n", pointCountTotal, exeTime, pntsPerSec);
    }

    for (int sz = 0; sz < outRenderResults.size(); ++sz)
    {
        timer.Start();

        if ((outRenderResults[sz].m_imageFileCacheOk == false) && 
            outRenderResults[sz].m_cloudPtr.get() && outRenderResults[sz].m_pointCount > 0)
        {
            RenderPointcloudToImage(renderer, outRenderResults[sz]);
            outRenderResults[sz].m_cloudPtr->Clear();

            timer.Stop();

            exeTime = timer.GetDurationInSecond();
            execExecTotal += exeTime;

            if (exeTime > 0.0)
            {
                utility::LogInfo("Pointcloud Render Duration: {} seconds\n", exeTime);
            }
        }
    }

    if (pointCountTotal > 0)
    {
        int pntsPerSec = (int)(pointCountTotal / execExecTotal);
        utility::LogInfo("Finished Loading {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}\n", pointCountTotal, execExecTotal, pntsPerSec);
    }

    delete renderer;

    return 1;
}

UINT RenderPointcloudToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo)
{
    const int width = 1440;
    const int height = 1080;
    int pointCount = 0;

    std::filesystem::path imagePath = fileInfo.m_FileName;
    imagePath = imagePath.replace_extension("jpg");

    fileInfo.m_ImageName = imagePath;

    pointCount = fileInfo.m_cloudPtr->points_.size();

    if (fileInfo.m_cloudPtr->HasPoints())
    {
        auto* scene = new Open3DScene(*modelRenderer);
        if (scene && fileInfo.m_cloudPtr)
        {

            auto pointcloud_mat = visualization::rendering::MaterialRecord();
            pointcloud_mat.shader = "defaultUnlit";
            pointcloud_mat.point_size = 4.0f;
            pointcloud_mat.base_color = { 0.5f, 0.5f, 0.5f, 1.0f };
            pointcloud_mat.absorption_color = { 0.4f, 0.4f, 0.4f };
            pointcloud_mat.emissive_color = { 0.4f, 0.4f, 0.4f, 1.0f };
            pointcloud_mat.sRGB_color = false;
            scene->SetLighting(Open3DScene::LightingProfile::NO_SHADOWS, { 0.5f, -0.5f, -0.5f });
            scene->GetScene()->EnableSunLight(true);
            scene->GetScene()->SetSunLightIntensity(35000);
            Eigen::Vector4f color = { 0.6, 0.6, 0.7, 1.0 };
            scene->SetBackground(color);
            scene->ShowAxes(false);

            scene->AddGeometry(fileInfo.m_FileName.string(), fileInfo.m_cloudPtr.get(), pointcloud_mat, true);

            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                float max_dim = float(0.3 * bounds.GetMaxExtent());
                Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                Eigen::Vector3f eye, up;

                eye = Eigen::Vector3f(center.x() + (max_dim / 1.2),
                                      center.y() + (max_dim / 1.2),
                                      center.z() + (max_dim / 1));
                up = Eigen::Vector3f(0, 0, 1);

                scene->GetCamera()->LookAt(center, eye, up);

                std::shared_ptr<geometry::Image> img;
                auto callback = [&img](std::shared_ptr<geometry::Image> _img)
                    {
                        img = _img;
                    };

                scene->GetView()->SetViewport(0, 0, width, height);

                modelRenderer->RenderToImage(scene->GetView(), scene->GetScene(), callback);
                modelRenderer->BeginFrame();
                modelRenderer->EndFrame();

                io::WriteImage(imagePath.string(), *img);

                utility::LogInfo("Writing image file: {}\n", imagePath.string());
            }

            delete scene;
        }
    }

    return pointCount;
}

UINT LoadPointCloudFile(NCraftImageGen::ImageGenResult& outLoadResults)
{
    int pointCount = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;

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
            pointCount = LoadLASorLAZToO3DCloud(outLoadResults.m_FileName, *new_cloud_ptr);
            outLoadResults.m_pointCount = pointCount;

            if (pointCount > 0)
            {
                new_cloud_ptr->SetName(outLoadResults.m_FileName.string());
                outLoadResults.m_cloudPtr = new_cloud_ptr;
            }
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
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    timer.Start();

    if (outLoadResults.size() == 1)
    {
        if (outLoadResults[0].m_imageFileCacheOk == false)
        {
            pointCount = LoadPointCloudFile(outLoadResults[0]);
            pointCountTotal = pointCount;
        }
    }
    else
    {

#pragma omp parallel for
        for (int sz = 0; sz < outLoadResults.size(); ++sz)
        {
            if (outLoadResults[sz].m_imageFileCacheOk == false)
            {
                MEMORYSTATUSEX statex;
                statex.dwLength = sizeof(statex);
                GlobalMemoryStatusEx(&statex);

                utility::Timer timer2;
                timer2.Start();

                pointCount = LoadPointCloudFile(outLoadResults[sz]);

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


UINT LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud)
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

//    utility::LogInfo("LAS...Point cloud points {}, thinning factor: {}, \n", lsHeader.pointCount(), pointStep);

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