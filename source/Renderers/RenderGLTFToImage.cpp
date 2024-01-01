#include "Renderers/RenderGLTFToImage.h"

namespace NCraftImageGen
{

UINT RenderModelsToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
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
            GetFileNamesFromDirectory(testPath, batchModeFilenames);
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
        }
    }

    if (batchModeFilenames.size() > 25)
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

        if (!reqPath.extension().compare(".gltf") ||
            !reqPath.extension().compare(".glb"))
        {
            toAddResult.m_modelType = 1;
        }

        outRenderResults.push_back(toAddResult);
    }

    utility::LogInfo("processing {} files....\n", batchModeFilenames.size());

    utility::Timer timer;
    double exeTime = 0.0, execExecTotal = 0.0;

    for (int sz = 0; sz < outRenderResults.size(); ++sz)
    {
        if (outRenderResults[sz].m_modelType == 1)
        {
            timer.Start();
            RenderModelToImage(renderer, outRenderResults[sz]);
            timer.Stop();

            exeTime = timer.GetDurationInSecond();
            execExecTotal += exeTime;

            outRenderResults[sz].m_processTimeSeconds = exeTime;

            utility::LogInfo("Model Load/Render Process Duration: {} seconds\n", exeTime);
        }
    }

    utility::LogInfo("Finished Loading {} files, Total Process Duration: {} seconds", outRenderResults.size(), execExecTotal);

    delete renderer;

    return 1;
}


UINT RenderModelToImage(FilamentRenderer* modelRenderer, NCraftImageGen::ImageGenResult& fileInfo)
{
    const int width = 1024;
    const int height = 768;
    bool model_success = false;
    visualization::rendering::TriangleMeshModel loaded_model;
    std::filesystem::path imagePath = fileInfo.m_FileName;

    imagePath = imagePath.replace_extension("jpg");

    fileInfo.m_ImageName = imagePath;

    try
    {
        io::ReadTriangleModelOptions opt;
        model_success = io::ReadTriangleModel(fileInfo.m_FileName.string(), loaded_model, opt);
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
            scene->AddModel(fileInfo.m_FileName.string(), loaded_model);

            scene->ShowAxes(false);

            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                scene->GetCamera()->CalcFarPlane(*scene->GetCamera(), bounds);
                scene->GetCamera()->CalcNearPlane();

                float max_dim = float(0.5 * bounds.GetMaxExtent());
                Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                Eigen::Vector3f eye, up;

                eye = Eigen::Vector3f(center.x() + (max_dim / 1.5f),
                                      center.y() + (max_dim / 1.0f),
                                      center.z() + (max_dim / 1.0f));
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
            }

            delete scene;
        }
    }

    return 1;
}

HBITMAP RenderModelToHBITMAP(std::filesystem::path& appPath, std::filesystem::path& filePath)
{
    HBITMAP result = NULL;
    const int width = 1024;
    const int height = 768;
    bool model_success = false;
    visualization::rendering::TriangleMeshModel loaded_model;
    std::filesystem::path imagePath = filePath;

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

    imagePath = imagePath.replace_extension("png");

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
        auto* scene = new Open3DScene(*renderer);

        if (scene)
        {
            scene->AddModel(filePath.string(), loaded_model);

            scene->ShowAxes(false);

            auto& bounds = scene->GetBoundingBox();

            if (bounds.GetMaxExtent() > 0.0f)
            {
                scene->GetCamera()->CalcFarPlane(*scene->GetCamera(), bounds);
                scene->GetCamera()->CalcNearPlane();

                float max_dim = float(0.65 * bounds.GetMaxExtent());
                Eigen::Vector3f center = bounds.GetCenter().cast<float>();
                Eigen::Vector3f eye, up;

                eye = Eigen::Vector3f(center.x() + (max_dim / 1.25f),
                                      center.y() + (max_dim / 1.0f),
                                      center.z() + (max_dim / 1.0f));
                up = Eigen::Vector3f(0, 1, 0);

                scene->GetCamera()->LookAt(center, eye, up);

                std::shared_ptr<geometry::Image> img;
                auto callback = [&img](std::shared_ptr<geometry::Image> _img)
                    {
                        img = _img;
                    };

                scene->GetView()->SetViewport(0, 0, width, height);

                renderer->RenderToImage(scene->GetView(), scene->GetScene(), callback);
                renderer->BeginFrame();
                renderer->EndFrame();

                io::WriteImage(imagePath.string(), *img);

                utility::LogInfo("Thumbnail: writing temp image file {}", imagePath.string().c_str());

                Gdiplus::Bitmap* bitmap = Gdiplus::Bitmap::FromFile(imagePath.c_str(), 0);
                if (bitmap)
                {
                    Gdiplus::Status status = bitmap->GetHBITMAP(NULL, &result);
                    if (result)
                    {
                        utility::LogInfo("got HBITMAP ok");
                    }
                     
                    delete bitmap;

                    std::filesystem::remove(imagePath);
                }
            }

            delete scene;
        }
    }

    return result;
}

}