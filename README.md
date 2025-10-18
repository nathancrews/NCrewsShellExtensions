NCrews Windows shell extension projects:

<b>LAS/LAZ Pointcloud Windows Shell Extension:</b> 
This "quick viewer" shell extension provides a Windows File Explorer right click menu option to quickly generate 3D preview images for large and small .las and .laz files.
(Disclaimer: Processing time depends on compute hardware) Typical processing time for 100mb .las is about 2 seconds. 25mb .laz file process in about the same times. Files are processed in parallel using 
available CPU cores. 
A second option will create a merged 3D image for all selected .las/laz files. 
Both menu options support multi-file select and directory processing.
<b>Pre-built Windows x64 installer available at</b> https://buymeacoffee.com/nathancrews/e/255641

<b>GLTF/GLB Windows Shell Extension:</b> 
This shell extension automatically adds thumbnail images to .glb files in Windows File Explorer and 
provides a right click menu option to generate a 3D preview image for .gltf files. Supports multi-file select and directory processing.
<b>Pre-built Windows x64 installer available at</b> https://buymeacoffee.com/nathancrews/e/255640

NOTE:
Projects use Open3D, OpenCV and LASLib C++ libraries to produce 3D images for the source content.
The library files are not included in this project, but are required to build as Dynamic DLLs.
Open3D: https://github.com/isl-org/Open3D
OpenCV: https://github.com/opencv/opencv
LASLib: https://github.com/LAStools/LAStools/tree/master/LASlib
