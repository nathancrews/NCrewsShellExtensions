#pragma once

#include <string>
#include <memory>
#include <vector>
#include "open3d/visualization/rendering/Model.h"

namespace NCrewsImageGen {

/// Helper class to extract textures from GLB files that ASSIMP doesn't handle
/// (e.g., WebP textures via EXT_texture_webp extension)
class GLBTextureExtractor {
public:
    /// Extract textures from a GLB file and populate the materials
    /// Returns true if any textures were successfully extracted
    static bool ExtractTextures(
        const std::string& glb_path,
        open3d::visualization::rendering::TriangleMeshModel& model);

private:
    struct GLBHeader {
        uint32_t magic;
        uint32_t version;
        uint32_t length;
    };

    struct GLBChunkHeader {
        uint32_t chunk_length;
        uint32_t chunk_type;
    };

    static constexpr uint32_t GLB_MAGIC = 0x46546C67; // "glTF"
    static constexpr uint32_t JSON_CHUNK_TYPE = 0x4E4F534A; // "JSON"
    static constexpr uint32_t BIN_CHUNK_TYPE = 0x004E4942; // "BIN\0"
};

} // namespace NCrewsImageGen
