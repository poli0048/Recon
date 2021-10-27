# Recon
![GitHub Screenshot](/Screenshots/Screenshot-MainScreen.png)
Recon is a C++ based, multi-vehicle ground control station (GCS) for collaborative drone imaging with cloud shadow prediction and avoidance capabilities. Recon has a high-level UI built on top of Dear ImGUI and a modular architecture for it's core components. It is currently only planned to support DJI vehicles through a companion iOS App (in development), but additional drones could/may be supported through additional modules in the future. **Note: Recon is a work in progress - it is not feature-complete.**

**High-Level Software Architecture:**
![GitHub Screenshot](/Doc/Software_Architecture_Diagram.png)

**Planned capabilities include:**
 * Control multiple drones at once and execute collaborative, multi-vehicle survey missions.
 * Drone collision avoidance built in at GCS level (in addition to built-in drone capabilities)
 * Built-in global GIS system for keep-out zones, avoidance zones, safe-landing zones, and min safe altitude. All layers can be viewed and edited directly from within Recon and are saved locally. This data is available to the Guidance Module - it is distinct from any DJI SDK-level restrictions, such as their cloud-based no-fly zone registry.
 * Live view of all drone states and drone locations on a satellite map (usable offline with on-disk caching)
 * With the "Shadow Detection" and "Shadow Propagation" modules, detect and track cloud shadows on the terrain. The Guidance module can incorporate shadow information into it's flight planning to minimize time spent in shadowed areas and ensure that each point on the ground is imaged at least once when not in shadow. Save a shadow map record showing where shadows were at each instant in a mission to aid in shadow-free image stitching.
 * Integrated vehicle simulator to support development and testing of different guidance algorithms 
 
**Modules:**
 * DJI Drone Interface Module: This module operates a network server and allows DJI drones to connect as clients (through a companion App running on an iOS device connected to the drones controller). When a drone is connected, this module exposes its telemetry, live imagery (if flying a supported camera), and command and control capabilities to other components of Recon. Note: Ideally we would define an abstract interface that could be implemented for various drone manufacturers, but this is unfortunately not practical due to major differences in control authority, accessible data, and command and control interfaces between manufacturers. If other drones are to be supported in the future they will require their own interfaces.
 * Shadow Detection Module: This module is capable of registering imagery from one of the drones equipped with a supported nadir-looking camera and fisheye lens and identifying cloud shadows on the terrain.
 * Shadow Propagation Module: This module takes instantaneous estimates of cloud shadows and predicts shadow evolution forward in time.
 * Guidance Module: Creates flight plans for the team of connected drones to collaboratively survey a given area. When using shadow detection, this module also periodically updates these plans to avoid shadowed regions.
 
# Contributors and License
Recon is developed by:
 * The University of Illinois Urbana-Champaign
 * Stanford University
 * The University of California, Berkeley
 * Sentek Systems, LLC

Recon is not a commercial product; it is developed as part of a USDA/NIFA research project: “NRI: FND: COLLAB: Multi-Vehicle Systems for Collecting Shadow-Free Imagery in Precision Agriculture” (NIFA Award # 2020-67021-30758). Recon is open source and distributed under a 3-clause BSD license (See "LICENSE"). It is not commercially supported and you use it at your own risk.

# Dependencies and Compatibility
Recon is designed and developed with a goal of maintaining platform independence as much as possible. A Linux Makefile is included for building on *NIX platforms. There isn't currently a build script included for Windows, but the core codebase and the dependencies chosen are all platform independent (many software components have already been tested when compiled with Visual Studio).

**Core Dependencies:**
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
 * [Dear ImGui (Malamanteau fork - "docking" branch)](https://github.com/malamanteau/imgui)
 * [implot](https://github.com/epezent/implot)
 * [HandyCPP](https://github.com/malamanteau/handycpp)
 * [RestClient-CPP](https://github.com/malamanteau/restclient-cpp)
 * [Native File Dialog](https://github.com/mlabbe/nativefiledialog)
 * [Flexible Raster Format](https://github.com/poli0048/Flexible-Raster-Format)
 * [Cereal](https://uscilab.github.io/cereal/)
 * [GLFW](https://github.com/glfw/glfw)
 * [FreeType](https://www.freetype.org/)
 * [LibCurl](https://curl.se/libcurl/)
 * [OpenCV4](https://opencv.org/)
 * [serial](https://github.com/wjwwood/serial)
 * [tacopie](https://github.com/Cylix/tacopie)
 * [SoLoud](https://github.com/jarikomppa/soloud/tree/master)

**Shadow Propagation Module Dependencies:**
 * [LibTorch](https://pytorch.org/)

# Building On Linux
You need GCC version 8 or newer to build Recon. Create a directory somewhere, let's call it "Repos". Clone the Recon repository into this directory (so this file has path "Repos/Recon/README.md"). Similarly clone the following dependencies into the Repos folder: Eigen, Dear ImGUI, implot, HandyCPP, RestClient-CPP, Native File Dialog, Flexible Raster Format, Cereal, GLFW, serial, tacopie, and SoLoud. These dependencies are all referenced using relative paths in the Recon project.

Next, GLFW needs to be compiled as follows:
 * Open a terminal to the GLFW directory
 * mkdir Release
 * cd Release
 * cmake -DCMAKE_BUILD_TYPE=Release -DGLFW_BUILD_DOCS=false -DGLFW_BUILD_EXAMPLES=false -DGLFW_BUILD_TESTS=false -S ../ -B .
 * make
 
This will create an archive that will be linked into Recon as part of the build process - note that you do not need to "make install" anything. **Important Note:** The given instructions for building GLFW may not work with versions of CMake prior to 3.13.4. If you run into problems you can build on top of the source directory and manually copy the file "libglfw3.a" to path Repos/glfw/Release/src/libglfw3.a.

Next, tacopie needs to be built as follows:
 * Open a terminal to the tacopie directory
 * mkdir build
 * cd build
 * cmake ..
 * make
 
This will create an archive that will be linked into Recon as part of the build process - note that you do not need to "make install" anything.

Next, use your package manager to ensure that you have the following libraries installed on your system (when available also install the "-dev" version): OpenGL, FreeType, LibCURL, GLEW, ALSA, and OpenCV (4.x). In Debian, you can get the needed dependencies (except currently OpenCV4) by installing the following packages: libglu1-mesa-dev, libglew-dev, freeglut3-dev mesa-common-dev, libfreetype6, libfreetype6-dev, libcurl4, libcurl4-openssl-dev, libasound2-dev.

If you are using a new enough distribution that OpenCV4 is available in your repos, you can use that (you will need the -dev version of all OpenCV packages). Otherwise, you need to build from source as follows. Download the latest stable version from Github. Then:
 * Open a terminal to the opencv directory
 * mkdir build
 * cd build
 * cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_GENERATE_PKGCONFIG=ON ..
 * make
 * sudo make install
 
 Note that if you build manually **you do need to "sudo make install" this dependency**.

We use a pre-built version of LibTorch (From the PyTorch project). Download the pre-built from here: [LibTorch 1.8.1](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.8.1%2Bcpu.zip). Unzip this in your Repos directory to a folder of the same name as the zip file (without the .zip extension). If you have done this correctly you should have a folder with path Repos/libtorch-cxx11-abi-shared-with-deps-1.8.1+cpu/libtorch/ that contains "bin", "include", "lib", and "share" sub-directories. You do not need to install anything; Recon will find the API headers and the Recon binary will know to load the dynamically linked libraries from the appropriate sub-directories at compile-time and at run-time, respectively.

Now open a terminal in the Recon directory and run "make". If the build succeeds, the binary program will end up in Repos/Recon/Bin.

# Building On Windows
TBD

# Current State
The main UI works. Satellite image retrieval and caching, along with the built-in GIS system are fully functional. Polygon support is working and the UI allows for the creation and editing of polygonal survey regions. Vehicle simulator is feature complete. UI-based control of real and simulated vehicles works. The shadow detection module is mostly complete (except for some UI integration). The NN-based shadow propagation module is partially complete but is not merged into the master branch yet. Guidance module work is ongoing. The iOS App bridge for controlling real drones is still in development but mostly functional (https://github.com/poli0048/Recon-DJI-IOS-Interface).



