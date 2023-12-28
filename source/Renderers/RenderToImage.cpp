#include "Renderers/RenderToImage.h"
#include <iostream>
#include <memory>
#include <thread>
#include <filesystem>

#pragma warning(push)
#pragma warning(disable: 4201 4324 4263 4264 4265 4266 4267 4512 4996 4702 4244 4250 4068)

#include "open3d/io/FileFormatIO.h"
#include "open3d/io/PointCloudIO.h"

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>

#include "pcl/PCLHeader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>

#include <tbb/tbb.h>
#include "tbb/concurrent_vector.h"
#include "tbb/concurrent_unordered_map.h"

using namespace pdal;
using namespace pdal::las;

namespace NCraftImageGen
{

std::vector<std::string> ModelFileExtensions{ ".gltf", ".glb",".GLTF", ".GLB" };
std::vector<std::string> PointcloudFileExtensions{ ".las",
".laz",
".pcd",
".xyz",
".pts",
".LAS",
".LAZ",
".PCD",
".XYZ",
".PTS" };

int RenderToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths)
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
            return RenderToImage(appPath, testPath);
        }
        else
        {
            for (std::string fext : ModelFileExtensions)
            {
                if (!testPath.extension().compare(fext))
                {
                    batchModeFilenames.push_back(testPath);
                    break;
                }
            }

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

#define DIV 1048576
// Use to convert bytes to MB
//#define DIV 1024

#define WIDTH 7

    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof(statex);
    GlobalMemoryStatusEx(&statex);
    utility::LogInfo("There is  {} percent of memory in use.\n", statex.dwMemoryLoad);
    utility::LogInfo("There are {} free Mbytes of physical memory.\n", statex.ullAvailPhys / DIV);
    utility::LogInfo("There are {} free Mbytes of virtual memory.\n", statex.ullAvailVirtual / DIV);

    //_tprintf(TEXT("There is  %*ld percent of memory in use.\n"), WIDTH, statex.dwMemoryLoad);
    //_tprintf(TEXT("There are %*I64d total Mbytes of physical memory.\n"), WIDTH, statex.ullTotalPhys / DIV);
    //_tprintf(TEXT("There are %*I64d free Mbytes of physical memory.\n"), WIDTH, statex.ullAvailPhys / DIV);
    //_tprintf(TEXT("There are %*I64d total Mbytes of paging file.\n"), WIDTH, statex.ullTotalPageFile / DIV);
    //_tprintf(TEXT("There are %*I64d free Mbytes of paging file.\n"), WIDTH, statex.ullAvailPageFile / DIV);
    //_tprintf(TEXT("There are %*I64d total Mbytes of virtual memory.\n"), WIDTH, statex.ullTotalVirtual / DIV);
    //_tprintf(TEXT("There are %*I64d free Mbytes of virtual memory.\n"), WIDTH, statex.ullAvailVirtual / DIV);
    //_tprintf(TEXT("There are %*I64d free Mbytes of extended memory.\n"), WIDTH, statex.ullAvailExtendedVirtual / DIV);


    utility::Timer timer;
    int pointCount = 0;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;
    tbb::concurrent_vector<std::shared_ptr<geometry::PointCloud>> cloudPtrs;
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    timer.Start();

#pragma omp parallel for
    for (int sz = 0; sz < batchModeFilenames.size(); ++sz)
    {
        MEMORYSTATUSEX statex;
        statex.dwLength = sizeof(statex);
        GlobalMemoryStatusEx(&statex);
        utility::LogInfo("There is  {} percent of memory in use.\n", statex.dwMemoryLoad);
        utility::LogInfo("There are {} free Mbytes of physical memory.\n", statex.ullAvailPhys / DIV);
        utility::LogInfo("There are {} free Mbytes of virtual memory.\n", statex.ullAvailVirtual / DIV);

        utility::Timer timer2;
        timer2.Start();

        if (!batchModeFilenames[sz].extension().compare(".las") ||
            !batchModeFilenames[sz].extension().compare(".laz") ||
            !batchModeFilenames[sz].extension().compare(".LAS") ||
            !batchModeFilenames[sz].extension().compare(".LAZ"))
        {
            std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();
            if (new_cloud_ptr)
            {
                pointCount = LoadLASorLAZToO3DCloud(batchModeFilenames[sz], *new_cloud_ptr);
                if (pointCount > 0)
                {
                    new_cloud_ptr->SetName(batchModeFilenames[sz].string());
                    cloudPtrs.push_back(new_cloud_ptr);
                }
            }
        }
        else if (!batchModeFilenames[sz].extension().compare(".pcd") ||
                 !batchModeFilenames[sz].extension().compare(".xyz") ||
                 !batchModeFilenames[sz].extension().compare(".pts"))
        {
            std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

            if (new_cloud_ptr)
            {
                io::ReadPointCloud(batchModeFilenames[sz].string(), *new_cloud_ptr);
                pointCount = new_cloud_ptr->points_.size();
                if (pointCount > 0)
                {
                    new_cloud_ptr->SetName(batchModeFilenames[sz].string());
                    cloudPtrs.push_back(new_cloud_ptr);
                }
            }
        }

        timer2.Stop();
        double exeTime = timer2.GetDurationInSecond();

        MEMORYSTATUSEX pStatex;
        pStatex.dwLength = sizeof(pStatex);
        GlobalMemoryStatusEx(&pStatex);

        utility::LogInfo("{} percent increase memory in use: {}\n", pStatex.dwMemoryLoad - statex.dwMemoryLoad, batchModeFilenames[sz].filename().string());
        utility::LogInfo("Loading used {} Mbytes of physical memory.\n", (statex.ullAvailPhys / DIV) - (pStatex.ullAvailPhys / DIV));

        if (pointCount > 0)
        {
            utility::LogInfo("Point cloud {}, points:{}, Threaded Load Duration: {} seconds\n", batchModeFilenames[sz].filename().string(), pointCount, exeTime);
        }

        omp_set_lock(&writelock);
        pointCountTotal += pointCount;
        omp_unset_lock(&writelock);
    }
    omp_destroy_lock(&writelock);

    timer.Stop();
    exeTime = timer.GetDurationInSecond();
    execExecTotal += exeTime;

    if (pointCountTotal > 0)
    {
        int pntsPerSec = (int)(pointCountTotal / exeTime);
        utility::LogInfo("==>Loaded {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}\n", pointCountTotal, exeTime, pntsPerSec);
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

        timer.Stop();

        exeTime = timer.GetDurationInSecond();
        execExecTotal += exeTime;

        if (pointCount > 0 && exeTime > 0.0)
        {
            utility::LogInfo("Pointcloud Render Duration: {} seconds\n", exeTime);
        }
    }

    utility::LogInfo("==>Total Load/Render for {} files, Duration: {} seconds\n", batchModeFilenames.size(), execExecTotal);

    delete renderer;

    return 1;
}


int RenderToImage(std::filesystem::path& appPath, std::filesystem::path& filePath)
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

    utility::Timer timer;
    int pointCount = 0;
    int pointCountTotal = 0;
    double execExecTotal = 0.0;
    double exeTime = 0.0;
    tbb::concurrent_vector<std::shared_ptr<geometry::PointCloud>> cloudPtrs;
    omp_lock_t writelock;
    omp_init_lock(&writelock);

    timer.Start();

#pragma omp parallel for
    for (int sz = 0; sz < batchModeFilenames.size(); ++sz)
    {
        utility::Timer timer2;
        timer2.Start();

        if (!batchModeFilenames[sz].extension().compare(".las") ||
            !batchModeFilenames[sz].extension().compare(".laz") ||
            !batchModeFilenames[sz].extension().compare(".LAS") ||
            !batchModeFilenames[sz].extension().compare(".LAZ"))
        {
            std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();
            if (new_cloud_ptr)
            {
                pointCount = LoadLASorLAZToO3DCloud(batchModeFilenames[sz], *new_cloud_ptr);
                if (pointCount > 0)
                {
                    new_cloud_ptr->SetName(batchModeFilenames[sz].string());
                    cloudPtrs.push_back(new_cloud_ptr);
                }
            }
        }
        else if (!batchModeFilenames[sz].extension().compare(".pcd") ||
                 !batchModeFilenames[sz].extension().compare(".xyz") ||
                 !batchModeFilenames[sz].extension().compare(".pts"))
        {
            std::shared_ptr<geometry::PointCloud> new_cloud_ptr = std::make_shared<geometry::PointCloud>();

            if (new_cloud_ptr)
            {
                io::ReadPointCloud(batchModeFilenames[sz].string(), *new_cloud_ptr);
                pointCount = new_cloud_ptr->points_.size();
                if (pointCount > 0)
                {
                    new_cloud_ptr->SetName(batchModeFilenames[sz].string());
                    cloudPtrs.push_back(new_cloud_ptr);
                }
            }
        }

        timer2.Stop();
        double exeTime = timer2.GetDurationInSecond();

        if (pointCount > 0)
        {
            utility::LogInfo("Point cloud {}, points:{}, Threaded Load Duration: {} seconds\n", batchModeFilenames[sz].filename().string(), pointCount, exeTime);
        }

        omp_set_lock(&writelock);
        pointCountTotal += pointCount;
        omp_unset_lock(&writelock);
    }
    omp_destroy_lock(&writelock);

    timer.Stop();
    exeTime = timer.GetDurationInSecond();
    execExecTotal += exeTime;

    if (pointCountTotal > 0)
    {
        int pntsPerSec = (int)(pointCountTotal / exeTime);
        utility::LogInfo("==>Loaded {} Total Points, Total Loading Process Duration: {} seconds, pnts/sec = {}\n", pointCountTotal, exeTime, pntsPerSec);
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

        timer.Stop();

        exeTime = timer.GetDurationInSecond();
        execExecTotal += exeTime;

        if (pointCount > 0 && exeTime > 0.0)
        {
            utility::LogInfo("Pointcloud Render Duration: {} seconds\n", exeTime);
        }
    }

    utility::LogInfo("==>Total Load/Render for {} files, Duration: {} seconds\n", batchModeFilenames.size(), execExecTotal);

    delete renderer;

    return 1;
}

int RenderModelToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath)
{
    const int width = 1024;
    const int height = 768;
    bool model_success = false;
    visualization::rendering::TriangleMeshModel loaded_model;
    std::filesystem::path imagePath = filePath;

    imagePath = imagePath.replace_extension("jpg");

    try
    {
        io::ReadTriangleModelOptions opt;
        model_success = io::ReadTriangleModel(filePath.string(), loaded_model, opt);
    }
    catch (...)
    {
        model_success = false;
        return 0;
    }

    if (model_success)
    {
        auto* scene = new Open3DScene(*modelRenderer);

        if (scene)
        {
            scene->AddModel(filePath.string(), loaded_model);

            scene->ShowAxes(false);

            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                scene->GetCamera()->CalcFarPlane(*scene->GetCamera(), bounds);
                scene->GetCamera()->CalcNearPlane();

                float max_dim = float(1.25 * bounds.GetMaxExtent());
                Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                Eigen::Vector3f eye, up;

                eye = Eigen::Vector3f(center.x() + (max_dim / 2),
                                      center.y() + (max_dim / 2),
                                      center.z() + (max_dim / 2));
                up = Eigen::Vector3f(0, 1, 0);

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

    return 1;
}

int RenderPointcloudToImage(FilamentRenderer* modelRenderer, std::filesystem::path& filePath, std::shared_ptr<geometry::PointCloud> new_cloud_ptr)
{
    const int width = 2048;
    const int height = 1640;
    int pointCount = 0;

    std::filesystem::path imagePath = filePath;
    imagePath = imagePath.replace_extension("jpg");

    pointCount = new_cloud_ptr->points_.size();

    if (new_cloud_ptr->HasPoints())
    {
        auto* scene = new Open3DScene(*modelRenderer);
        if (scene && new_cloud_ptr)
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
            Eigen::Vector4f color = { 0.0, 0.0, 0.0, 1.0 };
            scene->SetBackground(color);
            scene->ShowAxes(false);

            scene->AddGeometry(filePath.string(), new_cloud_ptr.get(), pointcloud_mat, true);

            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                float max_dim = float(1.25 * bounds.GetMaxExtent());
                Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                Eigen::Vector3f eye, up;

                eye = Eigen::Vector3f(center.x() + (max_dim / 3),
                                      center.y() + (max_dim / 3),
                                      center.z() + (max_dim / 4));
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

int LoadLASorLAZToO3DCloud(std::filesystem::path& fileName, geometry::PointCloud& pointcloud)
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

    if (lsHeader.pointCount() > pointToPixel)
    {
        pointStep = lsHeader.pointCount() / pointToPixel;
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

}
#pragma warning(pop)