//This module provides test benches that can be invoked through a command-line switch
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <tuple>
#include <string>
#include <vector>

namespace TestBenches {
	inline std::vector<std::string> AvailableTestBenches = {
		/*  0 */ "Geometry: Break line segments at intersections",
		/*  1 */ "Geometry: Simple Polygon Sanitize - Outline Trace",
		/*  2 */ "Geometry: Simple Polygon intersection fragmentation",
		/*  3 */ "Geometry: Polygon Triangulation",
		/*  4 */ "Survey Regions: Serialization",
		/*  5 */ "Survey Regions: Create Sample Minneapolis Region",
		/*  6 */ "Guidance: EstimateMissionTime() - Between 2 points",
		/*  7 */ "Guidance: ",
		/*  8 */ "Guidance: ",
		/*  9 */ "Guidance: ",
		/* 10 */ "Guidance: ",
		/* 11 */ "Shadow Detection: Non-realtime simulation",
		/* 12 */ "Shadow Detection: Realtime simulation",
		/* 13 */ "Shadow Detection: ",
		/* 14 */ "Shadow Detection: ",
		/* 15 */ "Shadow Detection: ",
		/* 16 */ "Shadow Propagation: Non-realtime simulation",
		/* 17 */ "Shadow Propagation: Realtime simulation",
		/* 18 */ "Shadow Propagation: ",
		/* 19 */ "Shadow Propagation: ",
		/* 20 */ "Shadow Propagation: ",
		/* 21 */ "DJI Drone Interface: Simulated Drone Imagery (Non-Realtime)",
		/* 22 */ "DJI Drone Interface: Simulated Drone Imagery (Realtime)",
		/* 23 */ "DJI Drone Interface: Serialization/Deserialization",
		/* 24 */ "DJI Drone Interface: Compressed Image Test",
		/* 25 */ "DJI Drone Interface: ",
		/* 26 */ "TorchLib basic testbench",
		/* 27 */ "Torchlib loading file"
	};
	
	void RunTestBench(int TestNum, std::string const & TestBenchArg);
}
