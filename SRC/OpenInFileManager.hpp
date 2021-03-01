//This header-only module provides a platform-independent way of opening an external file manager
//to a given location in a non-blocking manner.
#pragma once

//Platform Includes
#if defined (WIN32)
	#include "Windows.h"
	#include <Shellapi.h>
#endif

//External Includes
#include "../../handycpp/Handy.hpp" //Provides std::filesystem

inline bool OpenInFileManager(std::filesystem::path DirPath) {
	if ((! std::filesystem::exists(DirPath)) || (! std::filesystem::is_directory(DirPath)))
		return false;
	
	#if defined (__APPLE__) || defined(MACOSX)	// Apple / OSX
		return false;
	#elif defined (WIN32) // Windows
		ShellExecuteA(NULL, "open", DirPath.string().c_str(), NULL, NULL, SW_SHOWDEFAULT);
	#elif defined (BSD) // BSD variants
		return false;
	#elif defined (sun) || defined(__sun) // Solaris
		return false;
	#elif defined (__gnu_linux__)	// Linux
		std::string command = std::string("xdg-open '") + DirPath.string() + "' &";
		system(command.c_str());
		return true;
	#else
		return false;
	#endif
}
