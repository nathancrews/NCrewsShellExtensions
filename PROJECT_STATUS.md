# NCraft Image Generation - Project Status

## 2026-03-09

### Completed Tasks

#### WebP Texture Support in GLB/GLTF Rendering (2026-01-16)
- Created new `GLBTextureExtractor` class (`source/Renderers/GLBTextureExtractor.cpp/.h`)
  - Parses raw GLB binary format (header, JSON chunk, BIN chunk)
  - Detects and handles `EXT_texture_webp` extension for WEBP-encoded textures
  - Decodes WEBP, PNG, and JPEG textures directly from GLB buffer views
  - Extracts `baseColorTexture` and `metallicRoughnessTexture` into albedo, roughness, and metallic maps
  - Returns extracted textures as Open3D Image objects for use in rendering pipeline
- Updated `RenderGLTFToImage.cpp` to use new extractor
  - After model load, if albedo texture is missing, calls `GLBTextureExtractor::ExtractTextures`
  - Applied improved scene lighting: `SOFT_SHADOWS` profile, indirect light 36000, sun light 70000
  - Enabled ambient occlusion and anti-aliasing for higher quality output
  - Added guards to verify rendered image data before file write
  - Added optional debug logging path (`NCRAFT_ENABLE_TEXTURE_DEBUG` flag)
- Added sample GLB test files including `test-webp.glb` for validation
- Added installer backup files for both shell extensions (`*.back(21.7.1).aip`)
- Added individual `Register-Win11-GLTF.ps1` and `Register-Win11-PointCloud.ps1` scripts to installer `SetupFiles` directories
- Added `GLTF_EXTENSION_PROMO.md` promotional/feature description document

### Current Status
WebP texture support is complete. GLB files using `EXT_texture_webp` now render correctly with proper textures applied. Shell extension installers are up to date.

### Next Steps (Optional Future Work)
- Test WebP texture extraction against a broader range of real-world GLTF/GLB assets
- Consider adding `normalTexture` extraction to `GLBTextureExtractor` (currently omitted)
- Evaluate distributing updated installer packages to existing customers
- Consider scheduled task for periodic Windows 11 registry refresh (see prior entry)

---

## 2026-01-06

### Completed Tasks

#### Register-Win11-GLTF.ps1 Script Enhancement
- Modified script to support customer use case (installed extension only)
- Removed development build paths (no more searching for `build\Release\` directories)
- Script now exclusively uses installed DLL location: `C:\Program Files\NCrews Software\NCrews GLTF Shell Extension\ModelShellExtension.dll`
- Simplified error reporting to avoid confusing non-developer customers
- Added automatic thumbnail cache clearing at end of registration (single-run solution)
- Improved cache clearing logic with proper Explorer shutdown and restart handling
- Added pause at script exit to allow users to read error messages

#### Script Functionality Verification
- Confirmed script successfully handles ProgID associations
- Verified `Add-HandlerToProgId` function dynamically finds current ProgID and registers handler
- Confirmed script adds SystemFileAssociations entries to prevent context menu hiding
- Verified registry entries prevent other software installations from hiding shell extension menus
- Script uses SHChangeNotify to refresh Windows Shell after registration

#### Customer Deployment Ready
- Script is now production-ready for Windows 11 customers
- Single execution handles all necessary steps: DLL re-registration, registry configuration, cache clearing
- No additional customer steps required after running the script

### Current Status
Ready to distribute `Register-Win11-GLTF.ps1` to customers experiencing thumbnail generation issues on Windows 11.

### Next Steps (Optional Future Work)
- Consider adding scheduled task to periodically refresh registry entries (preventive measure for future software installations)
- Document script usage instructions for customer knowledge base
- Evaluate similar enhancement for Point Cloud extension script

---

## 2026-06-19

### Completed Tasks

#### Model format support rollout and registration updates
- Expanded model extension handling to include `.obj`, `.fbx`, and `.3mf` in the renderer extension pipeline.
- Updated shell registration paths in `ModelShellExtension` for context menu and thumbnail provider coverage for `.stl`, `.obj`, `.fbx`, and `.3mf`, including corresponding unregister paths.
- Extended Windows 11 registration script coverage so register/unregister/test/reset flows include `.obj`, `.fbx`, and `.3mf`.

#### Windows 10 registration and policy alignment
- Updated `Register-Win10-GLTF.ps1` into a dedicated Windows 10 registration workflow covering supported model extensions.
- Added explicit split between context menu support and thumbnail support in the Win10 script so `.gltf` remains available for context actions but is excluded from thumbnail registration.
- Added Win10 script diagnostics to clearly report intentionally unsupported thumbnail extensions.

#### Thumbnail behavior and regression hardening
- Enforced `.gltf` thumbnail rejection in `ModelThumbnail::GetThumbnail` to align with the single-file-access constraint for thumbnail handlers.
- Scoped GLB texture fallback path to `.glb` only in shared renderer logic to avoid broad behavior changes.

#### Build decoupling and stabilization
- Removed unintended CGI coupling introduced through shared renderer dependency changes.
- Added compile-time gating in `RenderGLTFToImage.cpp` (`NCRAFT_ENABLE_GLB_TEXTURE_EXTRACTOR`) so projects can opt out of GLB texture extractor compilation.
- Disabled GLB extractor path for `NCImageGen_CGI` and removed direct extractor source inclusion from CGI project files.
- Verified `ModelShellExtension` Release|x64 build succeeds after decoupling.

### Current Status
Model Shell Extension work is now scoped independently from CGI, current format-support registration changes are in place, and `.gltf` thumbnail generation is intentionally disabled by policy. The Model Shell Extension project builds successfully in Release|x64.

### Next Steps
- Run full installer-oriented validation for Windows 10 and Windows 11 registration scripts with real sample files across `.glb`, `.stl`, `.obj`, `.fbx`, and `.3mf`.
- Confirm right-click menu and thumbnail behavior for each supported extension in a clean user profile.
- Resolve remaining non-blocking compiler warning in `ModelThumbnail.cpp` (`ULONGLONG` to `ULONG`) during next cleanup pass.
