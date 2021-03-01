# Recon
![GitHub Screenshot](/Screenshots/Screenshot-MainScreen.png)
Recon is a C++ based, multi-vehicle ground control station (GCS) for collaborative drone imaging with cloud shadow prediction and avoidance capabilities. Recon has a high-level UI built on top of Dear ImGUI and a modular architecture for it's core components. It is currently only planned to support DJI vehicles through a companion iOS App (in development), but additional drones could/may be supported through additional modules in the future.

**Planned capabilities include:**
 * Control multiple drones at once and execute collaborative, multi-vehicle survey missions.
 * Drone collision avoidance built in at GCS level (in addition to built-in drone capabilities)
 * Built-in global GIS system for keep-out zones, avoidance zones, safe-landing zones, and min safe altitude. All layers can be viewed and edited directly from within Recon and are saved locally. This data is available to the Guidance Module - it is distinct from any DJI SDK-level restrictions, such as their cloud-based no-fly zone registry.
 * Live view of all drone states and drone locations on a satellite map (usable offline with on-disk caching)
 * With the "Shadow Detection" and "Shadow Propagation" modules, detect and track cloud shadows on the terrain. The Guidance module can incorporate shadow information into it's flight planning to minimize time spent in shadowed areas and ensure that each point on the ground is imaged at least once when not in shadow. Save a shadow map record showing where shadows were at each instant in a mission to aid in shadow-free image stitching.
 
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
 * [HandyCPP](https://github.com/malamanteau/handycpp)
 * [RestClient-CPP](https://github.com/malamanteau/restclient-cpp)
 * [Native File Dialog](https://github.com/mlabbe/nativefiledialog)
 * [Flexible Raster Format](https://github.com/poli0048/Flexible-Raster-Format)
 * [Cereal](https://uscilab.github.io/cereal/)
 * [GLFW](https://github.com/glfw/glfw)
 * [FreeType](https://www.freetype.org/)
 * [LibCurl](https://curl.se/libcurl/)

**DJI Drone Interface Module Dependencies:** TBD

**Shadow Detection Module Dependencies:**
 * [OpenCV](https://opencv.org/)

**Shadow Propagation Module Dependencies:** TBD

**Guidance Module Dependencies:** TBD

# Building On Linux
Create a directory somewhere, let's call it "Repos". Clone the Recon repository into this directory (so this file has path "Repos/Recon/README.md"). Similarly clone the following dependencies into the Repos folder: Eigen, Dear ImGUI, HandyCPP, RestClient-CPP, Native File Dialog, Flexible Raster Format, and Cereal. These dependencies are all referenced using relative paths in the Recon project. The Makefile will build each of them and statically link them in as part of the build process (some of them are header only and don't appear in the Makefile at all). Next, make sure you have GLFW, OpenGL, FreeType, and LibCURL development packages installed on your system. In it's current state this should be enough to build Recon. Open a terminal to the Recon directory and run "make". If the build succeeds, the binary program will end up in Repos/Recon/Bin.

# Building On Windows
TBD

# Current State
The main UI works. Satellite image retrieval and caching, along with the built-in GIS system are fully functional. Simple polygon support is working and the UI allows for the creation and editing of polygonal survey regions. Work is ongoing on the various modules but they are currently being developed as stand-alone projects and will be integrated later.



