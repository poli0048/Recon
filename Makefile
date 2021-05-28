#Linux Recon Makefile
#Author: Bryan Poling
#Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 

CC = gcc
#CC = g++
#CC = clang++-3.5

#Build type (Release or Debug) and Optimization level ("Optimized" means through AVX2 v.s. "Legacy" which only uses SSE2 and SSE3)
#Build types: Optimised-Release, Optimized-Debug, Legacy-Release, Legacy-Debug
BUILD_TYPE = Optimised-Release
#BUILD_TYPE = Legacy-Debug

#Set optimization flags
ifeq ($(BUILD_TYPE),$(filter $(BUILD_TYPE),Optimised-Release Optimised-Debug))
	OPTFLAGS1 = -ftree-vectorize -march=broadwell -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mavx -mfma -mbmi2 -mavx2
	OPTFLAGS2 = -mno-sse4a -mno-xop -mno-fma4 -mno-avx512f -mno-avx512vl -mno-avx512pf -mno-avx512er -mno-avx512cd
	OPTFLAGS3 = -mno-avx512dq -mno-avx512bw -mno-avx512ifma -mno-avx512vbmi -O3 -fopenmp
	OPTFLAGS  = $(OPTFLAGS1) $(OPTFLAGS2) $(OPTFLAGS3)
else
	OPTFLAGS1 = -ftree-vectorize -march=core2 -msse2 -msse3 -mssse3 -msse4.1 -mno-sse4.2 -mno-sse4a -mno-avx -mno-fma -mno-bmi2
	OPTFLAGS2 = -mno-avx2 -mno-xop -mno-fma4 -mno-avx512f -mno-avx512vl -mno-avx512pf -mno-avx512er -mno-avx512cd -mno-avx512dq
	OPTFLAGS3 = -mno-avx512bw -mno-avx512ifma -mno-avx512vbmi -Og -fopenmp
	OPTFLAGS =  $(OPTFLAGS1) $(OPTFLAGS2) $(OPTFLAGS3)
endif

#Set DEBUGFLAGS.
ifeq ($(BUILD_TYPE),$(filter $(BUILD_TYPE),Optimised-Debug Legacy-Debug))
	DEBUGFLAGS = -g
else
	DEBUGFLAGS = 
endif

# ****************************************************   Include Paths   ****************************************************
RECON_INCLUDE_FLAGS1 = -I.. -I../eigen -I../Flexible-Raster-Format -I../imgui -I../restclient-cpp/include -I../cereal/include
RECON_INCLUDE_FLAGS2 = `pkg-config --cflags freetype2 libcurl gtk+-3.0` -I../glfw/include -I../imgui/examples/libs/gl3w -I../nativefiledialog/src/include
RECON_INCLUDE_FLAGS3 = -I../libtorch-cxx11-abi-shared-with-deps-1.8.1+cpu/libtorch/include/
RECON_INCLUDE_FLAGS4 = -I../libtorch-cxx11-abi-shared-with-deps-1.8.1+cpu/libtorch/include/torch/csrc/api/include/
RECON_INCLUDE_FLAGS  = $(RECON_INCLUDE_FLAGS1) $(RECON_INCLUDE_FLAGS2) $(RECON_INCLUDE_FLAGS3) $(RECON_INCLUDE_FLAGS4)

# ****************************   Set C++ Standard, profiling and linker trim flags and Defines   ****************************
STANDARDFLAGS = -std=c++17
PROFILEFLAGS =
#PROFILEFLAGS = -pg
LINKER_TRIM_FLAGS = 
#LINKER_TRIM_FLAGS = -Wl,--gc-sections -Wl,--strip-all
DEFINE_FLAGS = -DGSL_USE_STD_BYTE -DLOADGLFWICON -DIMGUIAPP_USE_FAS

# ******************************************   Combine all Compile and Link Flags   *****************************************
COMPILE_WARNING_FLAGS = -Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-strict-aliasing \
                        -Wno-ignored-attributes -Wno-misleading-indentation
COMPILE_DEPGEN_FLAGS  = -MT $@ -MMD -MP -MF DEP/$(*F).d
ELF_FLAGS             = -fdata-sections -ffunction-sections
COMPILE_FLAGS         = -c -fdiagnostics-color=auto -pthread $(COMPILE_WARNING_FLAGS) $(RECON_INCLUDE_FLAGS) $(COMPILE_DEPGEN_FLAGS) \
                        $(DEBUGFLAGS) $(PROFILEFLAGS) $(LINKER_TRIM_FLAGS) $(DEFINE_FLAGS) $(STANDARDFLAGS) $(OPTFLAGS) $(ELF_FLAGS) `pkg-config --cflags opencv`
LINK_FLAGS            = -fdiagnostics-color=auto -static-libstdc++ -static-libgcc -lstdc++ -lstdc++fs \
                        -Wl,-Bdynamic -lpthread -lm -ldl -luuid -lGL -fopenmp \
                        `pkg-config --static --libs freetype2 libcurl gtk+-3.0` ../glfw/Release/src/libglfw3.a `pkg-config --libs opencv` \
                        -L ../libtorch-cxx11-abi-shared-with-deps-1.8.1+cpu/libtorch/lib/ -ltorch -ltorch_cpu -lc10 \
                        '-Wl,-rpath,$$ORIGIN/../../libtorch-cxx11-abi-shared-with-deps-1.8.1+cpu/libtorch/lib'

# **********************************************   Populate Source File Lists   *********************************************
#Populate source files that are part of Recon project
RECON_SRCFILES = $(wildcard SRC/*.cpp) $(wildcard SRC/UI/*.cpp) $(wildcard SRC/Maps/*.cpp) $(wildcard SRC/Modules/DJI-Drone-Interface/*.cpp) \
                 $(wildcard SRC/Modules/Guidance/*.cpp) $(wildcard SRC/Modules/Shadow-Detection/*.cpp) $(wildcard SRC/Modules/Shadow-Propagation/*.cpp)

#Build list of additional (external) source files.
EXTERNAL_SRCFILES = ../restclient-cpp/source/connection.cc \
                    ../restclient-cpp/source/helpers.cc \
                    ../restclient-cpp/source/restclient.cc \
                    ../imgui/imgui_widgets.cpp \
                    ../imgui/imgui_draw.cpp \
                    ../imgui/imgui_demo.cpp \
                    ../imgui/imgui.cpp \
                    ../imgui/misc/cpp/imgui_stdlib.cpp \
                    ../imgui/misc/freetype/imgui_freetype.cpp \
                    ../imgui/examples/imgui_impl_glfw.cpp \
                    ../imgui/examples/imgui_impl_opengl3.cpp \
                    ../imgui/examples/libs/gl3w/GL/gl3w.c \
                    ../imgui/app/ImGuiApp.cpp \
                    ../imgui/app/imconfig.cpp \
                    ../imgui/app/main_GL.cpp \
                    ../nativefiledialog/src/nfd_common.c \
                    ../nativefiledialog/src/nfd_gtk.c \
                    ../Flexible-Raster-Format/FRF.cpp \
                    ../handycpp/Handy.cpp

# **********************************************   Populate Object File Lists   *********************************************
RECON_OBJFILES    = $(patsubst SRC/%.cpp,OBJ/%.o,$(RECON_SRCFILES))
EXTERNAL_OBJFILES = $(addprefix OBJ/External/,$(addsuffix .o,$(basename $(notdir $(EXTERNAL_SRCFILES)))))
OBJFILES          = $(RECON_OBJFILES) $(EXTERNAL_OBJFILES)

# *****************************************************   Build Rules   *****************************************************

#Top-level rule (default build rule)
all: recon

recon: folders $(OBJFILES)
	$(CC) -Wall $(DEBUGFLAGS) $(PROFILEFLAGS) $(LINKER_TRIM_FLAGS) -L/usr/local/lib -o BIN/Recon $(OBJFILES) $(LINK_FLAGS)

#Reproduce the folder structure of SRC in OBJ and create an External sub-directory for external object files
folders:
	@mkdir -p DEP
	@mkdir -p OBJ/External
	@find SRC/ -type d -printf '%P\n' | sed 's/^/OBJ\//' | xargs mkdir -p

#Object file build rules
$(RECON_OBJFILES):
	@echo ""
	@echo "\033[1;34mCompiling file: \033[0m" $(shell echo $(RECON_SRCFILES) | grep -Eo '\<$(basename $(patsubst OBJ/%,SRC/%,$@))\.[^[:space:]]*|[[:space:]]$(basename $(patsubst OBJ/%,SRC/%,$@))\.[^[:space:]]*')
	@echo "\033[1;34mTarget Object:  \033[0m" $@
	$(CC) $(COMPILE_FLAGS) -o $@ $(shell echo $(RECON_SRCFILES) | grep -Eo '\<$(basename $(patsubst OBJ/%,SRC/%,$@))\.[^[:space:]]*|[[:space:]]$(basename $(patsubst OBJ/%,SRC/%,$@))\.[^[:space:]]*')

$(EXTERNAL_OBJFILES):
	@echo ""
	@echo "\033[1;34mCompiling file: \033[0m" $(shell echo $(EXTERNAL_SRCFILES) | grep -Eo '\<$(*F)\.[^[:space:]]*|[[:space:]]$(*F)\.[^[:space:]]*|[^[:space:]]*/$(*F)\.[^[:space:]]*')
	@echo "\033[1;34mTarget Object:  \033[0m" $@
	$(CC) $(COMPILE_FLAGS) -o $@ $(shell echo $(EXTERNAL_SRCFILES) | grep -Eo '\<$(*F)\.[^[:space:]]*|[[:space:]]$(*F)\.[^[:space:]]*|[^[:space:]]*/$(*F)\.[^[:space:]]*')

#Clean build rule    *************************************************************************************
clean:
	/bin/rm -r OBJ/*
	/bin/rm DEP/*.d
	/bin/rm -f BIN/Recon

#Dependency file include directive    ********************************************************************
-include DEP/*.d

#Variable inspection rule: make print-VAR
print-%  : ; @echo $* = $($*)




