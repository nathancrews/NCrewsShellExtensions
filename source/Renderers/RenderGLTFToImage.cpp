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
#include "GLBTextureExtractor.h"

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
        
        // If model loaded but has no textures, try extracting WebP textures manually
        if (model_success)
        {
            std::string file_ext = fileInfo.m_FileName.extension().string();
            std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);
            
            if (file_ext == ".glb" || file_ext == ".gltf")
            {
                // Check if materials exist but have no textures
                bool has_missing_textures = false;
                for (const auto& mat : loaded_model.materials_)
                {
                    if (!mat.albedo_img)
                    {
                        has_missing_textures = true;
                        break;
                    }
                }
                
                if (has_missing_textures)
                {
                    GLBTextureExtractor::ExtractTextures(fileInfo.m_FileName.string(), loaded_model);
                }
            }
        }
    }
    catch (...)
    {
        model_success = false;
        return 0;
    }

    if (model_success)
    {
        std::unique_ptr<Open3DScene> scene(new Open3DScene(*modelRenderer));

        if (scene)
        {
            scene->AddModel(fileInfo.m_FileName.string(), loaded_model);

            // Lighting: soften and brighten slightly (IBL + sun)
            scene->SetLighting(Open3DScene::LightingProfile::SOFT_SHADOWS,
                               Eigen::Vector3f(0.577f, -0.577f, -0.577f));
            // Fine-tune intensities
            scene->GetScene()->SetIndirectLightIntensity(36000.0f);
            scene->GetScene()->SetSunLightIntensity(70000.0f);
            scene->GetView()->SetAmbientOcclusion(true, true);
            scene->GetView()->SetAntiAliasing(true, true);

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

                if (img && img->HasData()) {
                    io::WriteImage(fileInfo.m_ImageName.string(), *img);
                } else {
                    utility::LogWarning("[TEXTURE DEBUG] RenderToImage produced null/empty image");
                    return 0;
                }
            }
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

    // Compile-time switch for debug logging (default OFF)
#ifndef NCRAFT_ENABLE_TEXTURE_DEBUG
#define NCRAFT_ENABLE_TEXTURE_DEBUG 0
#endif
#if NCRAFT_ENABLE_TEXTURE_DEBUG
    // Enable debug logging for texture loading to a file (use system temp directory)
    namespace fs = std::filesystem;
    const auto logPathFs = fs::temp_directory_path() / "Open3D_GLB_Debug.log";
    const std::string logPath = logPathFs.string();
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
    
    // Set log output to file
    if (FILE* logFile = fopen(logPath.c_str(), "a"))
    {
        fclose(logFile);
        utility::Logger::GetInstance().SetPrintFunction(
            [logPath](const std::string& msg) {
                if (FILE* f = fopen(logPath.c_str(), "a")) {
                    fprintf(f, "%s\n", msg.c_str());
                    fflush(f);
                    fclose(f);
                }
            });
    }
#else
    // Silence Open3D logs for production builds
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
#endif
    
    EngineInstance::SetResourcePath(resourcePath.string().c_str());

    std::unique_ptr<FilamentRenderer> renderer(
        new FilamentRenderer(EngineInstance::GetInstance(), width, height,
                             EngineInstance::GetResourceManager()));

    if (!renderer)
    {
        return 0;
    }

    imagePath = imagePath.replace_extension("png");

    utility::LogInfo("[TEXTURE DEBUG] Loading model from: {}", filePath.string());
    
    try
    {
        io::ReadTriangleModelOptions opt;
        model_success = io::ReadTriangleModel(filePath.string(), loaded_model, opt);
        
        if (model_success)
        {
            utility::LogInfo("[TEXTURE DEBUG] Model loaded successfully");
            utility::LogInfo("[TEXTURE DEBUG] Number of meshes: {}", loaded_model.meshes_.size());
            utility::LogInfo("[TEXTURE DEBUG] Number of materials: {}", loaded_model.materials_.size());
            
            // Try extracting WebP textures if missing
            std::string file_ext = filePath.extension().string();
            std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);
            
            if (file_ext == ".glb" || file_ext == ".gltf")
            {
                bool has_missing_textures = false;
                for (const auto& mat : loaded_model.materials_)
                {
                    if (!mat.albedo_img)
                    {
                        has_missing_textures = true;
                        break;
                    }
                }
                
                if (has_missing_textures)
                {
                    utility::LogInfo("[TEXTURE DEBUG] Attempting to extract WebP textures...");
                    GLBTextureExtractor::ExtractTextures(filePath.string(), loaded_model);
                }
            }
            
            // Log material details after extraction attempt
            for (size_t i = 0; i < loaded_model.materials_.size(); ++i)
            {
                auto& mat = loaded_model.materials_[i];
                utility::LogInfo("[TEXTURE DEBUG] Material {}: name='{}'", i, mat.name);
                utility::LogInfo("[TEXTURE DEBUG]   - Has albedo map: {}", mat.albedo_img ? "YES" : "NO");
                utility::LogInfo("[TEXTURE DEBUG]   - Has normal map: {}", mat.normal_img ? "YES" : "NO");
                utility::LogInfo("[TEXTURE DEBUG]   - Has roughness map: {}", mat.roughness_img ? "YES" : "NO");
                utility::LogInfo("[TEXTURE DEBUG]   - Has metallic map: {}", mat.metallic_img ? "YES" : "NO");
                if (mat.albedo_img)
                {
                    utility::LogInfo("[TEXTURE DEBUG]   - Albedo map size: {}x{}", 
                                     mat.albedo_img->width_, mat.albedo_img->height_);
                }
            }
        }
        else
        {
            utility::LogInfo("[TEXTURE DEBUG] Model loading FAILED");
        }
    }
    catch (const std::exception& e)
    {
        utility::LogInfo("[TEXTURE DEBUG] Exception loading model: {}", e.what());
        model_success = false;
        return 0;
    }
    catch (...)
    {
        utility::LogInfo("[TEXTURE DEBUG] Unknown exception loading model");
        model_success = false;
        return 0;
    }

    if (model_success)
    {
        auto* scene = new Open3DScene(*renderer);

        if (scene)
        {
            utility::LogInfo("[TEXTURE DEBUG] Adding model to scene...");
            scene->AddModel(filePath.string(), loaded_model);

            // Lighting tweaks to better match glTF viewer
            scene->SetLighting(Open3DScene::LightingProfile::SOFT_SHADOWS,
                               Eigen::Vector3f(0.577f, -0.577f, -0.577f));
            scene->GetScene()->SetIndirectLightIntensity(36000.0f);
            scene->GetScene()->SetSunLightIntensity(70000.0f);
            scene->GetView()->SetAmbientOcclusion(true, true);
            scene->GetView()->SetAntiAliasing(true, true);

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

                if (img && img->HasData()) {
                    io::WriteImage(imagePath.string(), *img);
                } else {
                    utility::LogWarning("[TEXTURE DEBUG] RenderToImage produced null/empty image");
                    return 0;
                }

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
                }

                // Best-effort cleanup of temp image file
                try { std::filesystem::remove(imagePath); } catch (...) {}
            }
        }

        delete scene;
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