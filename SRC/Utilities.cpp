//Generic utilities
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes

//External Includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//Project Includes
#include "Utilities.hpp"
#include "Modules/DJI-Drone-Interface/Drone.hpp"

cv::Mat GetRefFrame(std::filesystem::path const & DatasetPath) {
	std::filesystem::path refFramePath = DatasetPath / "RefFrame.jpeg"s;
	if (! std::filesystem::exists(refFramePath)) {
		std::cerr << "Error in GetRefFrame(): Source file does not exist.\r\n";
		std::cerr << "The file path is: " << DatasetPath.string() << "\r\n";
		return cv::Mat();
	}
	
	cv::Mat frame = cv::imread(refFramePath.string(), cv::IMREAD_COLOR);
	if (DroneInterface::SimulatedDrone::Resize_4K_to_720p(frame))
		return frame;
	else {
		std::cerr << "Error in GetRefFrame(): Resize_4K_to_720p() failed.\r\n";
		return cv::Mat();
	}
}

//Return the path of the first .MOV file in the given folder (lexicographically)
std::filesystem::path GetSimVideoFilePath(std::filesystem::path const & DatasetPath) {
	std::vector<std::filesystem::path> files = GetNormalFilesInDirectory(DatasetPath);
	std::sort(files.begin(), files.end(), [](std::string const & A, std::string const & B) -> bool { return StringNumberAwareCompare_LessThan(A, B); });
	for (auto const & file : files) {
		std::string ext = file.extension().string();
		if ((ext == ".mov"s) || (ext == ".MOV") || (ext == ".Mov"))
			return file;
	}
	return std::filesystem::path();
}

//Load GCPs from the file GCP.txt in the given folder - we also adjust them from file native res (4K) to 720p
//Returned fiducials are in form needed by Shadow Detection Engine.
std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> LoadFiducialsFromFile(std::filesystem::path const & DatasetPath) {
	double PI = 3.14159265358979;
	
	std::filesystem::path GCPsFilePath = DatasetPath / "GCPs.txt"s;
	std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> GCPs;
	if (! std::filesystem::exists(GCPsFilePath))
		return GCPs;
	
	std::ifstream file;
	file.open(GCPsFilePath.string().c_str());
	if (! file.good())
		return GCPs;
	
	std::string line;
	char character;
	size_t index;
	while (!file.eof()) {
		std::getline(file, line);
		index = line.find_first_not_of(" \t"s);
		if (index != std::string::npos) {
			//Line is non-empty. Check if it is a comment line
			character = line.at(index);
			if (character != '#') {
				//Line is non-empty and not a comment line
				line = line.substr(index);
				std::vector<std::string> parts = StringSplit(line, ","s);
				if (parts.size() != 6U)
					std::cerr << "Warning in LoadFiducials(): Dropping invalid line.\r\n";
				else {
					StringStrip(parts[5], " \t\r\n"); //Make sure newline or space chars don't mess us up at end of line
					double col, row, latDeg, lonDeg, altM;
					if (str2double(parts[1], col) && str2double(parts[2], row) && 
					    str2double(parts[3], latDeg) && str2double(parts[4], lonDeg) && str2double(parts[5], altM)) {
						Eigen::Vector2d PixCoords(col/3.0, row/3.0); //Convert 4K to 720p
						Eigen::Vector3d LLA(latDeg*PI/180.0, lonDeg*PI/180.0, altM); //Convert degrees to radians
						GCPs.push_back(std::make_tuple(PixCoords, LLA));
					}
					else
						std::cerr << "Warning: Dropping invalid GCP from file. Line: " << line << "\r\n";
				}
			}
		}
	}
	return GCPs;
}










