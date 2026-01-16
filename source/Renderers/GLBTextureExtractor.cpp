#include "GLBTextureExtractor.h"
#include <fstream>
#include "json.hpp"
#include "open3d/utility/Logging.h"
#include "open3d/io/ImageIO.h"
#include "open3d/geometry/Image.h"
#include "webp/decode.h"

using json = nlohmann::json;

namespace NCrewsImageGen {

bool GLBTextureExtractor::ExtractTextures(
    const std::string& glb_path,
    open3d::visualization::rendering::TriangleMeshModel& model)
{
    using namespace open3d;
    
    auto decode_to_image = [](const std::string& mime, const uint8_t* data, int len) -> std::shared_ptr<geometry::Image> {
        try {
            if (mime == "image/webp") {
                int w=0,h=0; unsigned char* decoded = WebPDecodeRGBA(data, len, &w, &h);
                if (decoded && w>0 && h>0) {
                    auto img = std::make_shared<geometry::Image>();
                    img->Prepare(w, h, 4, 1);
                    memcpy(img->data_.data(), decoded, size_t(w)*h*4);
                    WebPFree(decoded);
                    return img;
                }
                return nullptr;
            }
            // png / jpeg via Open3D
            std::string fmt = (mime == "image/png") ? "png" : (mime == "image/jpeg" ? "jpg" : "");
            if (!fmt.empty()) {
                return io::CreateImageFromMemory(fmt, data, len);
            }
        } catch (...) {}
        return nullptr;
    };

    std::ifstream file(glb_path, std::ios::binary);
    if (!file.is_open()) {
        utility::LogWarning("[GLBTextureExtractor] Could not open file: {}", glb_path);
        return false;
    }

    // Read GLB header
    GLBHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(GLBHeader));
    
    if (header.magic != GLB_MAGIC) {
        utility::LogWarning("[GLBTextureExtractor] Not a valid GLB file");
        return false;
    }

    utility::LogInfo("[GLBTextureExtractor] GLB version: {}, length: {}", 
                     header.version, header.length);

    // Read JSON chunk
    GLBChunkHeader json_chunk;
    file.read(reinterpret_cast<char*>(&json_chunk), sizeof(GLBChunkHeader));
    
    if (json_chunk.chunk_type != JSON_CHUNK_TYPE) {
        utility::LogWarning("[GLBTextureExtractor] Expected JSON chunk first");
        return false;
    }

    std::vector<char> json_data(json_chunk.chunk_length);
    file.read(json_data.data(), json_chunk.chunk_length);
    
    json gltf;
    try {
        gltf = json::parse(json_data.begin(), json_data.end());
    } catch (const std::exception& e) {
        utility::LogWarning("[GLBTextureExtractor] Failed to parse JSON: {}", e.what());
        return false;
    }

    // Read BIN chunk (if exists)
    GLBChunkHeader bin_chunk;
    file.read(reinterpret_cast<char*>(&bin_chunk), sizeof(GLBChunkHeader));
    
    if (bin_chunk.chunk_type != BIN_CHUNK_TYPE) {
        utility::LogWarning("[GLBTextureExtractor] No binary chunk found");
        return false;
    }

    std::vector<uint8_t> bin_data(bin_chunk.chunk_length);
    file.read(reinterpret_cast<char*>(bin_data.data()), bin_chunk.chunk_length);
    
    utility::LogInfo("[GLBTextureExtractor] Binary chunk size: {} bytes", bin_data.size());

    bool uses_webp = false;
    if (gltf.find("extensionsUsed") != gltf.end()) {
        for (const auto& ext : gltf["extensionsUsed"]) {
            if (ext == "EXT_texture_webp") { uses_webp = true; break; }
        }
    }
    if (uses_webp) utility::LogInfo("[GLBTextureExtractor] File uses EXT_texture_webp");

    // Extract textures
    bool extracted_any = false;
    
    if (gltf.find("materials") == gltf.end() || gltf.find("textures") == gltf.end() || 
        gltf.find("images") == gltf.end() || gltf.find("bufferViews") == gltf.end()) {
        utility::LogWarning("[GLBTextureExtractor] Missing required glTF components");
        return false;
    }

    auto& materials_json = gltf["materials"];
    auto& textures_json = gltf["textures"];
    auto& images_json = gltf["images"];
    auto& buffer_views_json = gltf["bufferViews"];

    utility::LogInfo("[GLBTextureExtractor] Found {} materials, {} textures, {} images",
                     materials_json.size(), textures_json.size(), images_json.size());

    // Process each material
    for (size_t mat_idx = 0; mat_idx < materials_json.size() && mat_idx < model.materials_.size(); ++mat_idx) {
        auto& mat_json = materials_json[mat_idx];
        auto& mat = model.materials_[mat_idx];

        utility::LogInfo("[GLBTextureExtractor] Processing material {}", mat_idx);

        // Extract baseColorTexture (albedo)
        if (mat_json.find("pbrMetallicRoughness") != mat_json.end()) {
            auto& pbr = mat_json["pbrMetallicRoughness"];
            
            if (pbr.find("baseColorTexture") != pbr.end()) {
                int texture_idx = pbr["baseColorTexture"]["index"];
                utility::LogInfo("[GLBTextureExtractor]   - baseColorTexture index: {}", texture_idx);

                // Get image index from texture
                if (texture_idx >= 0 && texture_idx < (int)textures_json.size()) {
                    auto& texture = textures_json[texture_idx];
                    
                    // Check for WebP extension
                    int image_idx = -1;
                    if (texture.find("extensions") != texture.end() && 
                        texture["extensions"].find("EXT_texture_webp") != texture["extensions"].end()) {
                        image_idx = texture["extensions"]["EXT_texture_webp"]["source"];
                        utility::LogInfo("[GLBTextureExtractor]     Using WebP image index: {}", image_idx);
                    } else if (texture.find("source") != texture.end()) {
                        image_idx = texture["source"];
                    }

                    // Extract image data
                    if (image_idx >= 0 && image_idx < (int)images_json.size()) {
                        auto& image = images_json[image_idx];
                        
                        if (image.find("bufferView") != image.end()) {
                            int buffer_view_idx = image["bufferView"];
                            std::string mime_type = image.value("mimeType", "");
                            
                            utility::LogInfo("[GLBTextureExtractor]     bufferView: {}, mimeType: {}", 
                                           buffer_view_idx, mime_type);

                            if (buffer_view_idx >= 0 && buffer_view_idx < (int)buffer_views_json.size()) {
                                auto& buffer_view = buffer_views_json[buffer_view_idx];
                                int byte_offset = buffer_view.value("byteOffset", 0);
                                int byte_length = buffer_view["byteLength"];

                                utility::LogInfo("[GLBTextureExtractor]     Extracting {} bytes from offset {}", 
                                               byte_length, byte_offset);

                                // Extract texture data
                                if (byte_offset + byte_length <= (int)bin_data.size()) {
                                    const uint8_t* texture_data = bin_data.data() + byte_offset;

                                    try {
                                        // Decode image to Open3D Image
                                        auto img = decode_to_image(mime_type, texture_data, byte_length);
                                        if (img) {
                                            mat.albedo_img = img;
                                            mat.sRGB_color = true;
                                            utility::LogInfo("[GLBTextureExtractor]     Decoded {} texture: {}x{}, {} channels", 
                                                           mime_type, img->width_, img->height_, img->num_of_channels_);
                                            extracted_any = true;
                                        } else {
                                            utility::LogWarning("[GLBTextureExtractor]     Failed to decode {} texture", mime_type);
                                        }
                                    } catch (const std::exception& e) {
                                        utility::LogWarning("[GLBTextureExtractor]     Exception decoding: {}", e.what());
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Extract metallicRoughnessTexture if needed
            if (pbr.find("metallicRoughnessTexture") != pbr.end()) {
                int texture_idx = pbr["metallicRoughnessTexture"]["index"];
                utility::LogInfo("[GLBTextureExtractor]   - metallicRoughnessTexture index: {}", texture_idx);
                
                if (texture_idx >= 0 && texture_idx < (int)textures_json.size()) {
                    auto& texture = textures_json[texture_idx];
                    int image_idx = -1;
                    if (texture.find("extensions") != texture.end() &&
                        texture["extensions"].find("EXT_texture_webp") != texture["extensions"].end()) {
                        image_idx = texture["extensions"]["EXT_texture_webp"]["source"];
                    } else if (texture.find("source") != texture.end()) {
                        image_idx = texture["source"];
                    }
                    if (image_idx >= 0 && image_idx < (int)images_json.size()) {
                        auto& image = images_json[image_idx];
                        if (image.find("bufferView") != image.end()) {
                            int buffer_view_idx = image["bufferView"];
                            std::string mime_type = image.value("mimeType", "");
                            if (buffer_view_idx >= 0 && buffer_view_idx < (int)buffer_views_json.size()) {
                                auto& buffer_view = buffer_views_json[buffer_view_idx];
                                int byte_offset = buffer_view.value("byteOffset", 0);
                                int byte_length = buffer_view["byteLength"];
                                utility::LogInfo("[GLBTextureExtractor]     (MR) Extracting {} bytes from offset {}", byte_length, byte_offset);
                                if (byte_offset + byte_length <= (int)bin_data.size()) {
                                    const uint8_t* texture_data = bin_data.data() + byte_offset;
                                    try {
                                        auto img = decode_to_image(mime_type, texture_data, byte_length);
                                        if (img && img->HasData()) {
                                            const int width = img->width_;
                                            const int height = img->height_;
                                            const int ch = img->num_of_channels_;
                                            const uint8_t* src = img->data_.data();
                                            size_t pixel_count = (size_t)width * height;
                                            
                                            mat.roughness_img = std::make_shared<open3d::geometry::Image>();
                                            mat.roughness_img->Prepare(width, height, 1, 1);
                                            mat.metallic_img = std::make_shared<open3d::geometry::Image>();
                                            mat.metallic_img->Prepare(width, height, 1, 1);
                                            unsigned char* dstR = mat.roughness_img->data_.data();
                                            unsigned char* dstM = mat.metallic_img->data_.data();
                                            
                                            if (ch >= 3) {
                                                for (size_t i = 0; i < pixel_count; ++i) {
                                                    dstR[i] = src[i*ch + 1]; // G
                                                    dstM[i] = src[i*ch + 2]; // B
                                                }
                                            } else if (ch == 2) {
                                                for (size_t i = 0; i < pixel_count; ++i) {
                                                    dstR[i] = src[i*2 + 0];
                                                    dstM[i] = src[i*2 + 1];
                                                }
                                            } else { // grayscale fallback
                                                memcpy(dstR, src, pixel_count);
                                                memset(dstM, 0, pixel_count);
                                            }
                                            extracted_any = true;
                                            utility::LogInfo("[GLBTextureExtractor]     Created roughness & metallic maps ({}x{}, ch={})", width, height, ch);
                                        } else {
                                            utility::LogWarning("[GLBTextureExtractor]     Failed to decode metallicRoughness texture");
                                        }
                                    } catch (const std::exception& e) {
                                        utility::LogWarning("[GLBTextureExtractor]     Exception decoding MR: {}", e.what());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Extract normalTexture if present
        if (mat_json.find("normalTexture") != mat_json.end()) {
            int texture_idx = mat_json["normalTexture"]["index"];
            utility::LogInfo("[GLBTextureExtractor]   - normalTexture index: {}", texture_idx);
            // Similar extraction logic (omitted for brevity)
        }
    }

    utility::LogInfo("[GLBTextureExtractor] Extraction complete. Success: {}", extracted_any);
    return extracted_any;
}

} // namespace NCrewsImageGen
