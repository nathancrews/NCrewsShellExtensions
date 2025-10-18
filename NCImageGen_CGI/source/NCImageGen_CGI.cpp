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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <sstream>
#include "inc/CGI_RenderWrapper.h"
#include <filesystem>

#pragma warning (push)
#pragma warning (disable: 4251)
#include "CgiUtils.h"
#include "CgiDefs.h"
#include "Cgicc.h"
#include "HTTPHTMLHeader.h"
#pragma warning (pop)

#include "Renderers/RenderToImageCommon.h"
#include "Renderers/RenderPointcloudToImage.h"
#include "Renderers/RenderGLTFToImage.h"

using namespace cgicc;

std::filesystem::path HandleLASFiles(std::filesystem::path& LASFilename, std::filesystem::path& exeDataPath, NCrewsImageGen::AppSettings& settings);
std::filesystem::path HandleGLTFFiles(std::filesystem::path& GLTFFilename, std::filesystem::path& exeDataPath, NCrewsImageGen::AppSettings& settings);

void Debug_log(const char* format, const char* value)
{

#if 0
    std::printf(format, value);
#endif

}

std::string GetParameterValue(std::string& inStr, const char* toFind)
{
    std::string retStr = "";

    if (inStr.find(toFind) == 0)
    {
        std::stringstream ss(inStr);
        std::string token;

        // get toFind
        std::getline(ss, token, '=');

        // get value of toFind after '='
        std::getline(ss, token, '=');

        retStr = token;
    }

    return retStr;
}


int main(int argc, char** argv, char** env)
{
    std::string header = "Content-type: text/html\r\n\r\n";
    std::cout << header;

    bool isPostRequest = false;
    std::filesystem::path doc_root, app_path, uploaded_file_name;
    std::string queryStr, fileStr, uploadedFileNameStr, requestMethodStr;
    bool doSubDirectories = false;
    std::filesystem::path LASFilename;
    std::filesystem::path LASImageFilename, LASImageFileURL;
    std::filesystem::path exeDataPath;

    for (; *argv != nullptr; argv++)
    {
        Debug_log("arg: %s\n", *argv);
    }

    Cgicc cgi;
    CgiEnvironment cgEnv = cgi.getEnvironment();

    queryStr = cgEnv.getQueryString();
    Debug_log("QUERY_STRING value: %s\n", queryStr.c_str());

    doc_root = cgEnv.getDocmentRoot();
    Debug_log("DOCUMENT_ROOT value: %s\n", doc_root.string().c_str());

    for (; *env != nullptr; env++)
    {
        std::string envStr = *env;
        if (envStr.find("CONTEXT_DOCUMENT_ROOT") == 0)
        {
            exeDataPath = GetParameterValue(envStr, "CONTEXT_DOCUMENT_ROOT");
            Debug_log("exeDataPath value: %s\n", exeDataPath.string().c_str());
            break;
        }
    }

    exeDataPath.make_preferred();

    fileStr = GetParameterValue(queryStr, "file");
    Debug_log("QueryStr:fileStr: %s\n", fileStr.c_str());
    LASFilename = fileStr;

    Debug_log("==>LASFilename: %s\n", LASFilename.string().c_str());

    bool isLASFileProcess = true;

    if (LASFilename.extension().string().compare(".las") &&
        LASFilename.extension().string().compare(".laz") &&
        LASFilename.extension().string().compare(".LAZ") &&
        LASFilename.extension().string().compare(".LAS"))
    {
        isLASFileProcess = false;
    }


    if (LASFilename.extension().string().compare(".las") &&
        LASFilename.extension().string().compare(".laz") &&
        LASFilename.extension().string().compare(".LAZ") &&
        LASFilename.extension().string().compare(".LAS") &&
        LASFilename.extension().string().compare(".gltf") &&
        LASFilename.extension().string().compare(".GLTF") &&
        LASFilename.extension().string().compare(".glb")  &&
        LASFilename.extension().string().compare(".GLB") &&
        LASFilename.extension().string().compare(".png") &&
        LASFilename.extension().string().compare(".jpg"))
    {
        std::cout << "<!DOCTYPE html><html lang = \"en\"><head><title>ImageGen Results - FAILED</title></head>"
            << "<body><h2><b>file type not supported:</b> " << LASFilename.string() << "</h2></body></html>" << "\n\n";

        return 0;
    }

    // make fully qualified input file path
    LASFilename = doc_root;

    if (isLASFileProcess)
    {
        LASFilename.append(las_default_dir);
    }
    else
    {
        LASFilename.append(gltf_default_dir);
    }

    LASFilename.append(fileStr);
    LASFilename.make_preferred();

    if (!std::filesystem::exists(LASFilename))
    {
        Debug_log("File: LASFilename: %s, not found\n", LASFilename.string().c_str());

        std::cout << "<!DOCTYPE html><html lang = \"en\"><head><title>ImageGen Results - FAILED</title></head>"
            << "<body><h2>file not found: " << LASFilename.string() << "</h2></body></html>" << "\n\n";

        return 0;
    }

    NCrewsImageGen::AppSettings imageGenSettings;

    LASImageFilename = LASFilename;
    LASImageFilename.replace_extension(imageGenSettings.imageFormat);

    Debug_log("app path: %s\n", exeDataPath.string().c_str());

    if (std::filesystem::exists(LASImageFilename))
    {
        Debug_log("Skipping Image Generation, file exists %s\n", LASImageFilename.string().c_str());
    }
    else
    {
        Debug_log("File: FileProcessName: %s, ok\n", LASFilename.string().c_str());

        if (isLASFileProcess)
        {
            LASImageFilename = HandleLASFiles(LASFilename, exeDataPath, imageGenSettings);
        }
        else
        {
            LASImageFilename = HandleGLTFFiles(LASFilename, exeDataPath, imageGenSettings);
        }
    }

    if (std::filesystem::exists(LASImageFilename))
    {
        std::string pathSep = "/";
        std::string upPath = "../";

        LASImageFileURL = fileStr;
        LASImageFileURL.replace_extension(imageGenSettings.imageFormat);

        Debug_log("Results ImageFileURL: %s\n", LASImageFileURL.string().c_str());

        std::string styleCSS = "<link rel=\"stylesheet\" href=\"styles/styles.css\"/>";
        std::string bootStrapCSS = "<link href=\"https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css\" rel=\"stylesheet\" integrity=\"sha384-QWTKZyjpPEjISv5WaRU9OFeRpok6YctnYmDr5pNlyT2bRjXh0JMhjY6hW+ALEwIH\" crossorigin=\"anonymous\">";

        if (isLASFileProcess)
        {
#ifdef BUILD_HTMX
            std::cout << "<html lang=\"en\"><head>" 
                << styleCSS
                << bootStrapCSS
                << "</head>"
                << "<body><img class=\"ImageResultPanel\" src=\""
                << upPath 
                << las_default_dir 
                << pathSep 
                << LASImageFileURL.string() << "\"/>"
                << "</body></html>";
#else
            std::cout << "<!DOCTYPE html><html lang = \"en\"><head><title>ImageGen Results</title></head>"
                << "<body><img src=\"" << upPath << las_default_dir << pathSep << LASImageFileURL.string() << "\"/>" << "</body></html>" << "\n\n";
#endif
        }
        else
        {
#ifdef BUILD_HTMX
            std::cout << "<html lang=\"en\"><head>"
                << styleCSS
                << bootStrapCSS
                << "</head>"
                << "<body><img class=\"ImageResultPanel\" src=\""
                << upPath 
                << gltf_default_dir 
                << pathSep 
                << LASImageFileURL.string() << "\"/>"
                << "</body></html>";
#else
            std::cout << "<!DOCTYPE html><html lang = \"en\"><head><title>ImageGen Results</title></head>"
                << "<body><img src=\"" << upPath << gltf_default_dir << pathSep << LASImageFileURL.string() << "\"/>" << "</body></html>" << "\n\n";
#endif
        }
    }
    else
    {
        std::cout << "<!DOCTYPE html><html lang = \"en\"><head><title>ImageGen Results - FAILED</title></head>"
            << "<body><h2>Error processing file: " << LASFilename.string() << "</h2></body></html>" << "\n\n";
    }

    return 0;
}

std::filesystem::path HandleLASFiles(std::filesystem::path& LASFilename, std::filesystem::path& exeDataPath, NCrewsImageGen::AppSettings& settings)
{
    std::filesystem::path retImageFilename;

    std::vector<std::filesystem::path> filesToImage;
    filesToImage.push_back(LASFilename);

    tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> renderResults;

    try
    {
        NCrewsImageGen::RenderPointcloudFiles(exeDataPath, filesToImage, settings, renderResults);
    }
    catch (...)
    {
        renderResults.clear();
    }

    if (renderResults.size() > 0)
    {
        retImageFilename = renderResults.front().m_ImageName;
    }

    return retImageFilename;
}


std::filesystem::path HandleGLTFFiles(std::filesystem::path& GLTFFilename, std::filesystem::path& exeDataPath, NCrewsImageGen::AppSettings& settings)
{
    std::filesystem::path retImageFilename;

    std::vector<std::filesystem::path> filesToImage;
    filesToImage.push_back(GLTFFilename);

    tbb::concurrent_vector<NCrewsImageGen::FileProcessPackage> renderResults;

    try
    {
        NCrewsImageGen::RenderModelsToImages(exeDataPath, filesToImage, settings, renderResults);
    }
    catch (...)
    {
        renderResults.clear();
    }

    if (renderResults.size() > 0)
    {
        retImageFilename = renderResults.front().m_ImageName;
    }

    return retImageFilename;
}