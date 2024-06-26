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


#include "Renderers/RenderGLTFToImage.h"

namespace NCrewsImageGen
{

UINT RenderModelsToImages(std::filesystem::path& appPath, std::vector<std::filesystem::path>& filePaths,
                          NCrewsImageGen::AppSettings& imageSettings,
                          tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage>& outRenderResults)
{
    open3d::utility::Logger::GetInstance().SetVerbosityLevel(utility::VerbosityLevel::Error);
    open3d::utility::Logger::GetInstance().SetPrintFunction(nullptr);

    const int width = imageSettings.imageWidth;
    const int height = imageSettings.imageHeight;

    std::filesystem::path resourcePath = appPath;
    resourcePath += L"resources";
    EngineInstance::SetResourcePath(resourcePath.string().c_str());

    std::vector<std::filesystem::path> batchModeFilenames;

    for (std::filesystem::path testPath : filePaths)
    {
        if (std::filesystem::is_directory(testPath))
        {
            GetFileNamesFromDirectory(testPath, ModelFileExtensions, batchModeFilenames);
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

    if (batchModeFilenames.empty())
    {
        return 0;
    }

    if (batchModeFilenames.size() > 25)
    {
        int retval = MessageBox(nullptr, L"    Proceed to Generate Images?", L"Info: Many files selected", MB_YESNO);

        if (retval != 6)
        {
            return 0;
        }
    }

    utility::LogInfo("processing {} files....", batchModeFilenames.size());

    for (std::filesystem::path reqPath : batchModeFilenames)
    {
        utility::LogInfo("Checking image cache for {}", reqPath.string().c_str());

        NCrewsImageGen::FileProcessPackage toAddResult(reqPath);

        uintmax_t fsize = 0;
        __std_win_error wep = std::filesystem::_File_size(reqPath, fsize);

        toAddResult.m_imageFileCacheOk = false;
        toAddResult.m_fileSize = fsize;
        toAddResult.m_modelType = 1;

        std::filesystem::file_time_type sourceFiletime = std::filesystem::last_write_time(reqPath);
        std::filesystem::path imagePath = reqPath;
        imagePath = imagePath.replace_extension(imageSettings.imageFormat);

        toAddResult.m_ImageName = imagePath;

        if (std::filesystem::exists(imagePath))
        {
            std::filesystem::file_time_type imageFiletime = std::filesystem::last_write_time(imagePath);

            if (sourceFiletime <= imageFiletime)
            {
                utility::LogInfo("Skipping image, file up to date: {}", imagePath.string().c_str());
                toAddResult.m_imageFileCacheOk = true;
            }
        }

        outRenderResults.push_back(toAddResult);
    }

    utility::Timer timer;
    double exeTime = 0.0, execExecTotal = 0.0;

    try
    {
        FilamentRenderer* renderer =
            new FilamentRenderer(EngineInstance::GetInstance(), width, height,
                                 EngineInstance::GetResourceManager());

        if (!renderer)
        {
            return 0;
        }

        for (int sz = 0; sz < outRenderResults.size(); ++sz)
        {
            if (outRenderResults[sz].m_imageFileCacheOk == false)
            {
                timer.Start();
                RenderModelToImage(renderer, imageSettings, outRenderResults[sz]);
                timer.Stop();

                exeTime = timer.GetDurationInSecond();
                execExecTotal += exeTime;

                outRenderResults[sz].m_processTimeSeconds = exeTime;
            }
            utility::LogInfo("Load/Render process duration for {}, {}s", outRenderResults[sz].m_FileName.string(), exeTime);
        }

        utility::LogInfo("Finished rendering {} files, Total Process Duration: {} seconds", outRenderResults.size(), execExecTotal);

        delete renderer;
    }
    catch (...)
    {
        utility::LogInfo("Load/Render process crashed..");
    }

    return 1;
}


UINT RenderModelToImage(FilamentRenderer* modelRenderer,
                        NCrewsImageGen::AppSettings& imageSettings, NCrewsImageGen::FileProcessPackage& fileInfo)
{
    const int width = imageSettings.imageWidth;
    const int height = imageSettings.imageHeight;
    bool model_success = false;
    visualization::rendering::TriangleMeshModel loaded_model;

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

                io::WriteImage(fileInfo.m_ImageName.string(), *img);
            }

            delete scene;
        }
    }

    return 1;
}

HBITMAP RenderModelToHBITMAP(std::filesystem::path& appPath,
                             NCrewsImageGen::AppSettings& imageSettings, std::filesystem::path& filePath)
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


int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
    UINT  num = 0;          // number of image encoders
    UINT  size = 0;         // size of the image encoder array in bytes

    ImageCodecInfo* pImageCodecInfo = NULL;

    GetImageEncodersSize(&num, &size);
    if (size == 0)
        return -1;  // Failure

    pImageCodecInfo = (ImageCodecInfo*)(malloc(size));
    if (pImageCodecInfo == NULL)
        return -1;  // Failure

    GetImageEncoders(num, size, pImageCodecInfo);

    for (UINT j = 0; j < num; ++j)
    {
        if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
        {
            *pClsid = pImageCodecInfo[j].Clsid;
            free(pImageCodecInfo);
            return j;  // Success
        }
    }

    free(pImageCodecInfo);
    return -1;  // Failure
}


}