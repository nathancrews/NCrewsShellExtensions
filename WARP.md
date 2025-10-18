# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

NCraft Image Generation is a Windows Shell Extension project that provides 3D visualization capabilities for point cloud (.las/.laz) and 3D model (.gltf/.glb) files. The project generates thumbnail previews and creates 3D images directly from Windows Explorer context menus.

## Architecture

### Core Components

- **CloudShellExtension** - Windows Shell Extension for point cloud files (.las/.laz)
- **ModelShellExtension** - Windows Shell Extension for 3D model files (.gltf/.glb) with thumbnail generation
- **NCraftImageGen** - Core rendering library (DLL) with shared functionality
- **NCImageGen_CGI** - Web CGI interface for server-side rendering
- **CloudExtensionOptions** - C# WinForms configuration tool for point cloud extension
- **GLTFExtensionOptions** - C# WinForms configuration tool for GLTF extension

### Key Libraries and Dependencies

- **Open3D** - 3D data processing and visualization (primary rendering engine)
- **OpenCV** - Image processing and output
- **LASLib** - LAS/LAZ point cloud file handling
- **TBB (Threading Building Blocks)** - Parallel processing
- **TinyXML** - XML configuration parsing
- **Cryptolens** - Licensing system
- **cgicc** - CGI web interface (CGI component only)

### File Structure

```
build/                      # Main build directory with solution and projects
├── NCraftImageGen.sln      # Main Visual Studio solution
├── *.vcxproj              # C++ project files for shell extensions
└── packages/              # NuGet packages

source/                    # C++ source code
├── CloudShellExtension/   # Point cloud shell extension implementation
├── ModelShellExtension/   # 3D model shell extension implementation
└── Renderers/             # Core rendering implementations

inc/                       # C++ header files
├── CloudShellExtension/   # Point cloud extension headers
├── ModelShellExtension/   # Model extension headers
└── Renderers/             # Rendering engine headers

NCImageGen_CGI/           # Web CGI application
CloudExtensionOptions/    # C# point cloud configuration app
GLTFExtensionOptions/     # C# GLTF configuration app
```

## Build System

### Configurations
- **Debug|x64** - Development builds with debug symbols
- **Release|x64** - Optimized production builds (primary target)
- **RelWithDebInfo|x64** - Release with debug information

### Build Commands

```bash
# Build entire solution (Release x64 - recommended)
msbuild build\NCraftImageGen.sln /p:Configuration=Release /p:Platform=x64

# Build specific components
msbuild build\CloudShellExtension.vcxproj /p:Configuration=Release /p:Platform=x64
msbuild build\ModelShellExtension.vcxproj /p:Configuration=Release /p:Platform=x64
msbuild CloudExtensionOptions\CloudExtensionOptions.csproj /p:Configuration=Release
msbuild GLTFExtensionOptions\GLTFExtensionOptions.csproj /p:Configuration=Release

# Clean solution
msbuild build\NCraftImageGen.sln /t:Clean
```

### Required Dependencies

External libraries must be installed at specific paths (hardcoded in project files):
- `D:\3rdpartylibs\Open3D18\` - Open3D library
- `D:\3rdpartylibs\opencv\` - OpenCV
- `D:\3rdpartylibs\LAStools-2.0.3\` - LAS processing
- `D:\3rdpartylibs\TBB\` - Intel TBB
- `D:\3rdpartylibs\cgicc-3.2.20\` - CGI library (for CGI component)

## Development Environment

### Prerequisites
- Visual Studio 2022 (v143 toolset)
- Windows 10 SDK
- .NET Framework 4.7.2 (for C# components)
- C++17 standard support

### Shell Extension Development
- Shell extensions require COM registration
- Use `regsvr32` for DLL registration/unregistration
- Debug by attaching to `explorer.exe` process
- Thumbnail cache clearing: use `Clear_and_Reset_Thumbnail_Cache.bat`

### Testing Workflow
1. Build shell extension DLLs
2. Register DLLs with Windows Shell
3. Test with sample files in `sample/` directory
4. Clear thumbnail cache if needed: `Clear_and_Reset_Thumbnail_Cache.bat`
5. Restart Windows Explorer: `re-explorer.bat`

## Key Features

### Point Cloud Processing
- Supports .las and .laz files
- Parallel processing using TBB
- Automatic camera positioning and scene setup
- Configurable image output resolution (default: 1440x1024)

### 3D Model Rendering
- GLTF/GLB model support with thumbnail generation
- Automatic Windows Explorer integration
- Context menu for manual preview generation

### CGI Web Interface
- Server deployment to `C:\Apache24\cgi-bin\` (configured in post-build)
- HTMX integration for web-based rendering
- JSON configuration support

## Configuration

### Application Settings
Settings stored in XML format under application data directories:
- Image dimensions (width/height)
- Output format (PNG default)
- License key management
- Processing parameters

### Licensing
- Cryptolens-based licensing system
- License validation for commercial features
- Settings stored in application data directories

## Deployment

### Shell Extension Installation
1. Build Release|x64 configuration
2. Copy DLLs to installation directory
3. Register with `regsvr32`
4. Copy required runtime DLLs (Open3D, OpenCV)

### Installer Packages
Pre-configured installers available:
- `build/NCrews Point Cloud Shell Extension-SetupFiles/` - Point cloud extension installer
- `build/NCrews GLTF Shell Extension-SetupFiles/` - GLTF extension installer

## Performance Considerations

- Point cloud processing is CPU-intensive, uses all available cores
- Typical processing time: ~2 seconds for 100MB .las files
- File processing supports batch operations
- Image caching reduces repeated processing overhead

## Troubleshooting

### Common Issues
- **Shell extension not appearing**: Check DLL registration with `regsvr32`
- **Thumbnails not updating**: Run `Clear_and_Reset_Thumbnail_Cache.bat`
- **Processing failures**: Verify required DLLs are in PATH or application directory
- **Build failures**: Ensure 3rd party library paths match hardcoded locations

### Windows 11 Specific Issues

#### Shell Extensions Not Working on Windows 11
Windows 11 has stricter requirements for shell extensions. Use the provided PowerShell scripts for proper registration:

```powershell
# Run as Administrator - Register both extensions
.\Register-Win11-Extensions.ps1           # Register all extensions
.\Register-Win11-Extensions.ps1 -ClearCache  # Clear thumbnail cache
.\Register-Win11-Extensions.ps1 -Test        # Test registration
.\Register-Win11-Extensions.ps1 -Unregister  # Unregister extensions

# Individual extension scripts
.\Register-Win11-GLTF.ps1              # GLTF/GLB files only
.\Register-Win11-PointCloud.ps1         # Point cloud files only (.las/.laz)
```

#### Key Windows 11 Fixes Applied

**GLTF/GLB Extension:**
1. **Multiple Interface Support**: Added proper `IInitializeWithFile` and `IInitializeWithItem` implementations
2. **Enhanced Registry Registration**: Added Windows 11 specific registry entries  
3. **Thumbnail Provider Registration**: Added additional thumbnail provider GUID registration
4. **Alpha Channel Support**: Proper `WTS_ALPHATYPE` handling in `GetThumbnail`
5. **Error Handling**: Enhanced exception handling and validation
6. **Cache Management**: Forced thumbnail cache refresh during registration

**Point Cloud Extension:**
1. **COM Interface Casting**: Fixed unsafe COM interface casting with proper static_cast
2. **Registry Compatibility**: Enhanced registry registration with HKLM/HKCU fallback
3. **Resource Management**: Improved memory management and null pointer checks
4. **Error Handling**: Added comprehensive exception handling and validation
5. **File Path Validation**: Enhanced file existence checks before processing
6. **Shell Notifications**: Added Windows 11 specific shell change notifications

#### Manual Windows 11 Verification Steps
1. Build the Release|x64 configuration
2. Run PowerShell script as Administrator
3. Copy a .glb/.gltf file to desktop
4. Enable thumbnail view in File Explorer
5. Verify custom thumbnails appear
6. Test right-click context menu

### Debug Utilities
- `re-explorer.bat` - Restart Windows Explorer
- `Clear_and_Reset_Thumbnail_Cache.bat` - Reset thumbnail cache
- `Register-Win11-Extensions.ps1` - Master Windows 11 registration script (both extensions)
- `Register-Win11-GLTF.ps1` - GLTF/GLB Windows 11 registration script
- `Register-Win11-PointCloud.ps1` - Point cloud Windows 11 registration script
- Visual Studio debugger attachment to `explorer.exe` for shell extension debugging
