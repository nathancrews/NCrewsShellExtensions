#include "Renderers/RenderToImageCommon.h"

namespace NCraftImageGen
{

std::vector<std::string> ModelFileExtensions{ ".gltf", ".glb",".GLTF", ".GLB" };
std::vector<std::string> PointcloudFileExtensions{ ".las",
".laz",
".pcd",
".ply",
".LAS",
".LAZ",
".PCD",
".PLY" };


UINT GetFileNamesFromDirectory(std::filesystem::path& filePath, std::vector<std::filesystem::path>& outDirectoryFilenames)
{
    UINT fileCount = 0;

    // directory for multiple file requested
    if (std::filesystem::is_directory(filePath))
    {
        for (auto const& dir_entry :
             std::filesystem::recursive_directory_iterator(filePath))
        {
            if (dir_entry.is_regular_file())
            {
                for (std::string fext : ModelFileExtensions)
                {
                    if (!dir_entry.path().extension().compare(fext))
                    {
                        outDirectoryFilenames.push_back(dir_entry.path());
                        break;
                    }
                }

                for (std::string pcext : PointcloudFileExtensions)
                {
                    if (!dir_entry.path().extension().compare(pcext))
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
