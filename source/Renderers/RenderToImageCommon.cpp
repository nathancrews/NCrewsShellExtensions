#include "Renderers/RenderToImageCommon.h"
#include "tinyxml2.h"

namespace NCraftImageGen
{

std::vector<std::string> ModelFileExtensions{ ".gltf", ".glb",".GLTF", ".GLB" };
std::vector<std::string> PointcloudFileExtensions{ ".las",
".laz",
".pcd",
".ply",
".pts",
".xyz",
".LAS",
".LAZ",
".PCD",
".PLY",
".PTS",
".XYZ" };


bool ReadImageGenSettings(std::filesystem::path& appDataPath, ImageGenSettings& outSettings)
{
    bool retval = false;

    if (!std::filesystem::exists(appDataPath))
    {
        return retval;
    }

    tinyxml2::XMLDocument settingsDoc;

    if (tinyxml2::XMLError::XML_SUCCESS != settingsDoc.LoadFile(appDataPath.string().c_str()))
    {
        return retval;
    }

    tinyxml2::XMLElement* root = settingsDoc.RootElement();

    if (root)
    {
        tinyxml2::XMLElement* settingsElem = root->FirstChildElement("Settings");

        if (settingsElem)
        {
            tinyxml2::XMLElement* imageFormatElem = settingsElem->FirstChildElement("ImageFormat");
            if (imageFormatElem && imageFormatElem->FirstChild())
            {
                outSettings.imageFormat = imageFormatElem->FirstChild()->Value();
            }

            tinyxml2::XMLElement* imageSizeElem = settingsElem->FirstChildElement("ImageSize");
            if (imageSizeElem && imageSizeElem->FirstChild())
            {
                retval = true;

                std::string sizeStr = imageSizeElem->FirstChild()->Value();

                if (!sizeStr.compare("1024x768"))
                {
                    outSettings.imageWidth = 1024;
                    outSettings.imageHeight = 768;
                }
                else if (!sizeStr.compare("1440x1024"))
                {
                    outSettings.imageWidth = 1440;
                    outSettings.imageHeight = 1024;
                }
                else if (!sizeStr.compare("2048x1440"))
                {
                    outSettings.imageWidth = 2048;
                    outSettings.imageHeight = 1440;
                }
            }

            tinyxml2::XMLElement* keyElem = settingsElem->FirstChildElement("LicenseKey");
            if (keyElem && keyElem->FirstChild())
            {
                outSettings.licenseKey = keyElem->FirstChild()->Value();
                if (outSettings.licenseKey.length() == 23)
                {
                    outSettings.is_Licensed = true;
                }
            }
        }

        settingsDoc.Clear();
    }

    return retval;
}


UINT GetFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::string>& allowedFileExtensions, std::vector<std::filesystem::path>& outDirectoryFilenames)
{
    UINT fileCount = 0;

    // directory for multiple file requested
    if (std::filesystem::is_directory(filePath))
    {
        for (auto const& dir_entry :
             std::filesystem::directory_iterator(filePath))
        {
            if (dir_entry.is_regular_file())
            {
                for (std::string fext : allowedFileExtensions)
                {
                    if (!dir_entry.path().extension().compare(fext))
                    {
                        outDirectoryFilenames.push_back(dir_entry.path());
                        break;
                    }
                }
            }
        }
    }

    fileCount = outDirectoryFilenames.size();

    return fileCount;
}

UINT GetCloudFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::string>& allowedFileExtensions, std::vector<std::filesystem::path>& outDirectoryFilenames)
{
    UINT fileCount = 0;

    // directory for multiple file requested
    if (std::filesystem::is_directory(filePath))
    {
        for (auto const& dir_entry :
             std::filesystem::directory_iterator(filePath))
        {
            if (dir_entry.is_regular_file())
            {
                for (std::string fext : allowedFileExtensions)
                {
                    if (!dir_entry.path().extension().compare(fext))
                    {
                        outDirectoryFilenames.push_back(dir_entry.path());
                        break;
                    }
                }
            }
        }
    }

    fileCount = outDirectoryFilenames.size();

    return fileCount;
}

}
