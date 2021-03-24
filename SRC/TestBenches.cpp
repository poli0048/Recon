//This module provides test benches that can be invoked through a command-line switch
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//System Includes
//#include <tuple>
#include <iostream>
#include <chrono>

//External Includes
#include "../../handycpp/Handy.hpp"
#include <opencv2/opencv.hpp>
//#include <opencv2/core/persistence.hpp>

//Project Includes
#include "TestBenches.hpp"
#include "Polygon.hpp"
#include "SurveyRegionManager.hpp"
#include "Maps/MapUtils.hpp"
#include "Modules/Guidance/Guidance.hpp"
#include "Modules/DJI-Drone-Interface/DroneManager.hpp"

#define PI 3.14159265358979

// Local function declarations (static linkage)
static bool TestBench0(void);   static bool TestBench1(void);
static bool TestBench2(void);   static bool TestBench3(void);
static bool TestBench4(void);   static bool TestBench5(void);
static bool TestBench6(void);   static bool TestBench7(void);
static bool TestBench8(void);   static bool TestBench9(void);
static bool TestBench10(void);  static bool TestBench11(void);
static bool TestBench12(void);  static bool TestBench13(void);
static bool TestBench14(void);  static bool TestBench15(void);
static bool TestBench16(void);  static bool TestBench17(void);
static bool TestBench18(void);  static bool TestBench19(void);
static bool TestBench20(void);  static bool TestBench21(void);
static bool TestBench22(void);  static bool TestBench23(void);
static bool TestBench24(void);  static bool TestBench25(void);

// ************************************************************************************************************************************************
// *********************************************************   Public Function Definitions   ******************************************************
// ************************************************************************************************************************************************

namespace TestBenches {
	void RunTestBench(int TestNum) {
		if ((TestNum < 0) || (TestNum >= (int) AvailableTestBenches.size())) {
			std::cerr << "Invalid testbench index. Aborting.\r\n";
			return;
		}
		std::cerr << "Running Test number " << TestNum << "\r\n";
		std::cerr << "TestBench description: " << AvailableTestBenches[TestNum] << "\r\n";
		bool result = false;
		switch (TestNum) {
			case 0:  result = TestBench0();  break;
			case 1:  result = TestBench1();  break;
			case 2:  result = TestBench2();  break;
			case 3:  result = TestBench3();  break;
			case 4:  result = TestBench4();  break;
			case 5:  result = TestBench5();  break;
			case 6:  result = TestBench6();  break;
			case 7:  result = TestBench7();  break;
			case 8:  result = TestBench8();  break;
			case 9:  result = TestBench9();  break;
			case 10: result = TestBench10(); break;
			case 11: result = TestBench11(); break;
			case 12: result = TestBench12(); break;
			case 13: result = TestBench13(); break;
			case 14: result = TestBench14(); break;
			case 15: result = TestBench15(); break;
			case 16: result = TestBench16(); break;
			case 17: result = TestBench17(); break;
			case 18: result = TestBench18(); break;
			case 19: result = TestBench19(); break;
			case 20: result = TestBench20(); break;
			case 21: result = TestBench21(); break;
			case 22: result = TestBench22(); break;
			case 23: result = TestBench23(); break;
			case 24: result = TestBench24(); break;
			case 25: result = TestBench25(); break;
			default: break;
		}
		if (result)
			std::cerr << "Testbench passed.\r\n";
		else
			std::cerr << "Testbench failed.\r\n";
	}
}

// ************************************************************************************************************************************************
// ********************************************************   Internal Function Definitions   *****************************************************
// ************************************************************************************************************************************************

static bool TestBench0(void) {
	std::Evector<LineSegment> InputSegments;
	InputSegments.emplace_back(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0));
	InputSegments.emplace_back(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0));
	std::Evector<LineSegment> OutputSegments = BreakAtIntersections(InputSegments);
	std::cerr << "Input Segments:\r\n";
	for (auto const & item : InputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "Output Segments:\r\n";
	for (auto const & item : OutputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "\r\n";
	
	InputSegments.clear();
	InputSegments.emplace_back(Eigen::Vector2d(0.0,  0.0), Eigen::Vector2d(1.0, 0.0));
	InputSegments.emplace_back(Eigen::Vector2d(0.01, 0.0), Eigen::Vector2d(0.0, 1.0));
	OutputSegments = BreakAtIntersections(InputSegments);
	std::cerr << "Input Segments:\r\n";
	for (auto const & item : InputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "Output Segments:\r\n";
	for (auto const & item : OutputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "\r\n";
	
	InputSegments.clear();
	InputSegments.emplace_back(Eigen::Vector2d(0.0,  0.0), Eigen::Vector2d(5.0,  0.0));
	InputSegments.emplace_back(Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.0,  1.0));
	InputSegments.emplace_back(Eigen::Vector2d(0.0,  1.0), Eigen::Vector2d(1.0, -1.0));
	InputSegments.emplace_back(Eigen::Vector2d(1.0, -1.0), Eigen::Vector2d(2.0,  1.0));
	InputSegments.emplace_back(Eigen::Vector2d(2.0,  1.0), Eigen::Vector2d(3.0, -1.0));
	InputSegments.emplace_back(Eigen::Vector2d(3.0, -1.0), Eigen::Vector2d(3.0,  1.0));
	OutputSegments = BreakAtIntersections(InputSegments);
	std::cerr << "Input Segments:\r\n";
	for (auto const & item : InputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "Output Segments:\r\n";
	for (auto const & item : OutputSegments)
		std::cerr << item << "\r\n";
	std::cerr << "\r\n";
	
	return true;
}

static bool TestBench1(void) {
	SimplePolygon poly;
	std::Evector<Eigen::Vector2d> vertices;
	
	//Test if simple polygon is left unchanged
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	poly.SetBoundary(vertices);
	std::cerr << "Sanitized Simple Polygon:\r\n" << poly << "\r\n";
	
	//Test if simple polygon is left unchanged
	vertices.clear();
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.1);
	vertices.emplace_back(1.1, 1.1);
	vertices.emplace_back(0.1, 0.9);
	poly.SetBoundary(vertices);
	std::cerr << "Sanitized Simple Polygon:\r\n" << poly << "\r\n";
	
	//Test order reversal of simple polygon
	vertices.clear();
	vertices.emplace_back(0.0, 1.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(0.5, 0.0);
	vertices.emplace_back(0.0, 0.0);
	poly.SetBoundary(vertices);
	std::cerr << "Sanitized Simple Polygon:\r\n" << poly << "\r\n";
	
	//Test order reversal of simple polygon
	vertices.clear();
	vertices.emplace_back(1.1, 1.1);
	vertices.emplace_back(1.0, 0.1);
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(0.1, 0.9);
	poly.SetBoundary(vertices);
	std::cerr << "Sanitized Simple Polygon:\r\n" << poly << "\r\n";
	
	//Test tracing for simple self-intersecting polygon
	vertices.clear();
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	vertices.emplace_back(1.0, 0.0);
	poly.SetBoundary(vertices);
	std::cerr << "Sanitized Simple Polygon:\r\n" << poly;
	std::cerr << "Area: " << poly.GetArea() << "\r\n";
	
	//Test winding-number polygon inclusion test
	Eigen::Vector2d P;
	P << 0, 0;
	std::cerr << "(" << P(0) << ", " << P(1) << ") is " << (poly.ContainsPoint(P) ? "in"s : "not in"s) << " polygon.\r\n";
	P << -0.01, 0;
	std::cerr << "(" << P(0) << ", " << P(1) << ") is " << (poly.ContainsPoint(P) ? "in"s : "not in"s) << " polygon.\r\n";
	P << 0.01, 0.005;
	std::cerr << "(" << P(0) << ", " << P(1) << ") is " << (poly.ContainsPoint(P) ? "in"s : "not in"s) << " polygon.\r\n";
	P << 0.5, 0.25;
	std::cerr << "(" << P(0) << ", " << P(1) << ") is " << (poly.ContainsPoint(P) ? "in"s : "not in"s) << " polygon.\r\n";
	P << 0.0, 0.5;
	std::cerr << "(" << P(0) << ", " << P(1) << ") is " << (poly.ContainsPoint(P) ? "in"s : "not in"s) << " polygon.\r\n";
	
	std::cerr << "\r\n";
	
	return true;
}

static bool TestBench2(void) {
	SimplePolygon polyA, polyB;
	std::Evector<Eigen::Vector2d> vertices;
	
	//Simple Test - polygon with itself
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	polyA.SetBoundary(vertices);
	polyB.SetBoundary(vertices);
	std::cerr << "*** Before Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	polyA.FragmentIntersections(polyB);
	std::cerr << "*** After Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	
	//Simple Test
	vertices.clear();
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	polyA.SetBoundary(vertices);
	vertices.clear();
	vertices.emplace_back(0.5, 0.25);
	vertices.emplace_back(1.5, 0.25);
	vertices.emplace_back(1.5, 0.75);
	vertices.emplace_back(0.5, 0.75);
	polyB.SetBoundary(vertices);
	std::cerr << "*** Before Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	polyA.FragmentIntersections(polyB);
	std::cerr << "*** After Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	
	//Simple Test
	vertices.clear();
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	polyA.SetBoundary(vertices);
	vertices.clear();
	vertices.emplace_back(0.5, 0.0);
	vertices.emplace_back(1.5, 0.0);
	vertices.emplace_back(1.5, 1.0);
	vertices.emplace_back(0.5, 1.0);
	polyB.SetBoundary(vertices);
	std::cerr << "*** Before Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	polyA.FragmentIntersections(polyB);
	std::cerr << "*** After Fragmentation ***\r\n";
	std::cerr << "Polygon A:\r\n" << polyA << "\r\n";
	std::cerr << "Polygon B:\r\n" << polyB << "\r\n";
	
	return true;
}

static bool TestBench3(void) {
	Polygon poly;
	std::Evector<Eigen::Vector2d> vertices;
	vertices.emplace_back(0.0, 0.0);
	vertices.emplace_back(1.0, 0.0);
	vertices.emplace_back(1.0, 1.0);
	vertices.emplace_back(0.0, 1.0);
	poly.m_boundary.SetBoundary(vertices);
	
	std::Evector<Triangle> Triangles;
	poly.Triangulate(Triangles);
	std::cerr << "Triangulation:\r\n";
	for (auto const & triangle : Triangles)
		std::cerr << triangle << "\r\n";
	std::cerr << "\r\n";
	
	vertices.clear();
	vertices.emplace_back(0.2, 0.2);
	vertices.emplace_back(0.8, 0.2);
	vertices.emplace_back(0.8, 0.8);
	vertices.emplace_back(0.2, 0.8);
	poly.m_holes.emplace_back();
	poly.m_holes.back().SetBoundary(vertices);
	
	Triangles.clear();
	poly.Triangulate(Triangles);
	std::cerr << "Triangulation:\r\n";
	for (auto const & triangle : Triangles)
		std::cerr << triangle << "\r\n";
	std::cerr << "\r\n";
	
	return true;
	
}

static bool TestBench4(void) {
	std::Evector<Eigen::Vector2d> vertices;
	
	{
		SurveyRegion region;
		region.m_Name = "Test Region"s;
		
		vertices.clear();
		vertices.emplace_back(0.0, 0.0);
		vertices.emplace_back(1.0, 0.0);
		vertices.emplace_back(1.0, 1.0);
		vertices.emplace_back(0.0, 1.0);
		region.m_Region.m_components.emplace_back();
		region.m_Region.m_components.back().m_boundary.SetBoundary(vertices);
		
		vertices.clear();
		vertices.emplace_back(0.2, 0.2);
		vertices.emplace_back(0.8, 0.2);
		vertices.emplace_back(0.8, 0.8);
		vertices.emplace_back(0.2, 0.8);
		region.m_Region.m_components.back().m_holes.emplace_back();
		region.m_Region.m_components.back().m_holes.back().SetBoundary(vertices);
		
		vertices.clear();
		vertices.emplace_back(2.0, 0.0);
		vertices.emplace_back(3.0, 0.0);
		vertices.emplace_back(2.5, 1.5);
		region.m_Region.m_components.emplace_back();
		region.m_Region.m_components.back().m_boundary.SetBoundary(vertices);
		//On destruction, region should be saved to disk automatically
	}
	
	{
		SurveyRegion region("Test Region"s); //Should load from disk
		std::cerr << "Region Contents:\r\n";
		std::cerr << region.m_Region;
		std::cerr << "\r\n\r\n";
		std::cerr << "Triangulation:\r\n";
		for (auto const & t : region.m_triangulation)
			std::cerr << t << "\r\n";
		std::cerr << "\r\n";
	}
	
	return true;
	
}

static bool TestBench5(void) {
	std::Evector<Eigen::Vector2d> vertices;
	
	{
		SurveyRegion region;
		region.m_Name = "Minneapolis Sample Region"s;
		
		vertices.clear();
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982718, -93.290769)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982199, -93.290775)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982212, -93.290120)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.981684, -93.289543)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.981813, -93.288494)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982791, -93.288633)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982671, -93.289731)));
		region.m_Region.m_components.emplace_back();
		region.m_Region.m_components.back().m_boundary.SetBoundary(vertices);
		
		vertices.clear();
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982401, -93.289944)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982255, -93.289962)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982259, -93.289604)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982439, -93.289628)));
		region.m_Region.m_components.back().m_holes.emplace_back();
		region.m_Region.m_components.back().m_holes.back().SetBoundary(vertices);
		
		vertices.clear();
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.981965, -93.291486)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.981958, -93.292436)));
		vertices.emplace_back(LatLonToNM(PI/180.0*Eigen::Vector2d(44.982602, -93.291921)));
		region.m_Region.m_components.emplace_back();
		region.m_Region.m_components.back().m_boundary.SetBoundary(vertices);
		
		//On destruction, region should be saved to disk automatically
	}
	
	return true;
}

static bool TestBench6(void) {
	//First test bench for guidance module - setup test case and call EstimateMissionTime() for mission time flying between two points
	DroneInterface::Waypoint A;
	DroneInterface::Waypoint B;
	double TargetSpeed = 15.0; //Max mission speed for DJI drones being controlled through SDK (m/s)
	
	//First point about 100 feet above the ground in Lamberton test area
	A.Latitude  =  0.772121618310453;
	A.Longitude = -1.663470726988503;
	A.Altitude  = 380.0;
	
	//Second point about 100 feet above the ground in Lamberton test area
	B.Latitude  =  0.772065994667192;
	B.Longitude = -1.663426622518305;
	B.Altitude  = 380.0;
	
	double time = Guidance::EstimateMissionTime(A, B, TargetSpeed);
	std::cerr << "Estimated flight fime from point A to point B (stopped at beginning and end) is: " << time << " seconds.\r\n";
	
	return false; //Ideally check result somehow and return true on success and false on failure (otherwise manually verify result somehow)
}

static bool TestBench7(void)  { return false; }
static bool TestBench8(void)  { return false; }
static bool TestBench9(void)  { return false; }
static bool TestBench10(void) { return false; }

//Shadow Detection: Non-realtime simulation
static bool TestBench11(void) {
	//Set up drone sim
	DroneInterface::Drone * myDrone = DroneInterface::DroneManager::Instance().GetDrone("Simulation"s);
	if (myDrone == nullptr) {
		std::cerr << "Error: Unable to get simulated drone from drone manager.\r\n";
		return false;
	}
	DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(myDrone);
	if (mySimDrone == nullptr) {
		std::cerr << "Error: Could not down-cast Drone to SimulatedDrone.\r\n";
		return false;
	}
	mySimDrone->SetRealTime(false);
	mySimDrone->SetSourceVideoFile(Handy::Paths::ThisExecutableDirectory() / "SimSourceVideo.mov"s);
	
	//Get reference frame - this is provided by the drone sim, but must come after setting the video file path
	cv::Mat refFrame;
	if (! mySimDrone->GetReferenceFrame(1.0, refFrame)) {
		std::cerr << "Error: Failed to get reference frame from SimulatedDrone.\r\n";
		return false;
	}
	
	//Get fiducials from file - we use Ben's file formats
	cv::FileStorage GCP_LLA_FileStoreObj((Handy::Paths::ThisExecutableDirectory() / "Sim_GCPs_LLA.xml"s).string(), cv::FileStorage::READ);
	cv::FileStorage GCP_PixCoords_FileStoreObj((Handy::Paths::ThisExecutableDirectory() / "Sim_GCPs_PixCoords.xml"s).string(), cv::FileStorage::READ);
	if ((! GCP_LLA_FileStoreObj.isOpened()) || (! GCP_PixCoords_FileStoreObj.isOpened())) {
		std::cerr << "Error: Unable to read Ground Control Points (GCPs) from disk.\r\n";
		std::cerr << "Latitude, Longitude, and Altitude are read from file: Sim_GCPs_LLA.xml\r\n";
		std::cerr << "Pixel coordinates are read from file ---------------: Sim_GCPs_PixCoords.xml\r\n";
		return false;
	}
	cv::Mat fiducials_LLA; //Stored row-by-row (Lat (deg), Lon (deg), Alt (m))
	std::vector<cv::Point2d> fiducials_PX; //OpenCV defs for X, Y... units are pixels
	GCP_LLA_FileStoreObj["gcp"] >> fiducials_LLA;
	GCP_PixCoords_FileStoreObj["pixel_coords"] >> fiducials_PX;
	
	//Convert fiducials to format expected by the shadow detection engine
	std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> Fiducials;
	if (fiducials_LLA.rows != int(fiducials_PX.size())) {
		std::cerr << "GCP size mismatch (we should have as many Lat/Lon/Alts as we have pixel coords)\r\n";
		return false;
	}
	Fiducials.reserve(fiducials_PX.size());
	for (int row = 0; row < fiducials_LLA.rows; row++) {
		Eigen::Vector2d PixCoords(fiducials_PX[row].x, fiducials_PX[row].y);
		Eigen::Vector3d LLA(fiducials_LLA.at<double>(row, 0)*PI/180.0, fiducials_LLA.at<double>(row, 1)*PI/180.0, fiducials_LLA.at<double>(row, 2));
		Fiducials.push_back(std::make_tuple(PixCoords, LLA));
	}
	
	//Set reference frame and fiducials in shadow detection module
	ShadowDetection::ShadowDetectionEngine::Instance().SetReferenceFrame(refFrame);
	ShadowDetection::ShadowDetectionEngine::Instance().SetFiducials(Fiducials);
	
	//Register callback with the shadow detection engine for monitoring its output (if desired)
	bool showLiveOutput = false;
	if (showLiveOutput) {
		ShadowDetection::ShadowDetectionEngine::Instance().RegisterCallback([](ShadowDetection::InstantaneousShadowMap const & ShadowMap) {
			auto duration = ShadowMap.Timestamp.time_since_epoch();
			double secondsSinceEpoch = double(duration.count()) * double(std::chrono::system_clock::period::num) / double(std::chrono::system_clock::period::den);
			std::cerr << "Shadow map received. Timestamp: " << secondsSinceEpoch << "\r\n";
			cv::imshow("Live Shadow Map", ShadowMap.Map);
			cv::waitKey(1);
		});
	}
	
	//Start the shadow detection engine, using imagery from the sim drone
	ShadowDetection::ShadowDetectionEngine::Instance().Start("Simulation"s);
	
	//Start the sim drone video feed
	myDrone->StartDJICamImageFeed(1.0);
	
	//wait until video feed is done
	while (! mySimDrone->IsSimVideoFinished())
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	//Stop the shadow detection engine and instruct it to save it's shadow map history
	ShadowDetection::ShadowDetectionEngine::Instance().Stop();
	ShadowDetection::ShadowDetectionEngine::Instance().SaveAndFlushShadowMapHistory();
	std::cerr << "Shadow map history saved to FRF files. Look in 'Shadow Map Files' folder in BIN directory.\r\n";
	
	return true;
}

//Shadow Detection: Realtime simulation
static bool TestBench12(void) {
	//Set up drone sim
	DroneInterface::Drone * myDrone = DroneInterface::DroneManager::Instance().GetDrone("Simulation"s);
	if (myDrone == nullptr) {
		std::cerr << "Error: Unable to get simulated drone from drone manager.\r\n";
		return false;
	}
	DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(myDrone);
	if (mySimDrone == nullptr) {
		std::cerr << "Error: Could not down-cast Drone to SimulatedDrone.\r\n";
		return false;
	}
	mySimDrone->SetRealTime(true);
	mySimDrone->SetSourceVideoFile(Handy::Paths::ThisExecutableDirectory() / "SimSourceVideo.mov"s);
	
	//Get reference frame - this is provided by the drone sim, but must come after setting the video file path
	cv::Mat refFrame;
	if (! mySimDrone->GetReferenceFrame(1.0, refFrame)) {
		std::cerr << "Error: Failed to get reference frame from SimulatedDrone.\r\n";
		return false;
	}
	
	//Get fiducials from file - we use Ben's file formats
	cv::FileStorage GCP_LLA_FileStoreObj((Handy::Paths::ThisExecutableDirectory() / "Sim_GCPs_LLA.xml"s).string(), cv::FileStorage::READ);
	cv::FileStorage GCP_PixCoords_FileStoreObj((Handy::Paths::ThisExecutableDirectory() / "Sim_GCPs_PixCoords.xml"s).string(), cv::FileStorage::READ);
	if ((! GCP_LLA_FileStoreObj.isOpened()) || (! GCP_PixCoords_FileStoreObj.isOpened())) {
		std::cerr << "Error: Unable to read Ground Control Points (GCPs) from disk.\r\n";
		std::cerr << "Latitude, Longitude, and Altitude are read from file: Sim_GCPs_LLA.xml\r\n";
		std::cerr << "Pixel coordinates are read from file ---------------: Sim_GCPs_PixCoords.xml\r\n";
		return false;
	}
	cv::Mat fiducials_LLA; //Stored row-by-row (Lat (deg), Lon (deg), Alt (m))
	std::vector<cv::Point2d> fiducials_PX; //OpenCV defs for X, Y... units are pixels
	GCP_LLA_FileStoreObj["gcp"] >> fiducials_LLA;
	GCP_PixCoords_FileStoreObj["pixel_coords"] >> fiducials_PX;
	
	//Convert fiducials to format expected by the shadow detection engine
	std::Evector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> Fiducials;
	if (fiducials_LLA.rows != int(fiducials_PX.size())) {
		std::cerr << "GCP size mismatch (we should have as many Lat/Lon/Alts as we have pixel coords)\r\n";
		return false;
	}
	Fiducials.reserve(fiducials_PX.size());
	for (int row = 0; row < fiducials_LLA.rows; row++) {
		Eigen::Vector2d PixCoords(fiducials_PX[row].x, fiducials_PX[row].y);
		Eigen::Vector3d LLA(fiducials_LLA.at<double>(row, 0)*PI/180.0, fiducials_LLA.at<double>(row, 1)*PI/180.0, fiducials_LLA.at<double>(row, 2));
		Fiducials.push_back(std::make_tuple(PixCoords, LLA));
	}
	
	//Set reference frame and fiducials in shadow detection module
	ShadowDetection::ShadowDetectionEngine::Instance().SetReferenceFrame(refFrame);
	ShadowDetection::ShadowDetectionEngine::Instance().SetFiducials(Fiducials);
	
	//Register callback with the shadow detection engine for monitoring its output (if desired)
	bool showLiveOutput = false;
	if (showLiveOutput) {
		ShadowDetection::ShadowDetectionEngine::Instance().RegisterCallback([](ShadowDetection::InstantaneousShadowMap const & ShadowMap) {
			auto duration = ShadowMap.Timestamp.time_since_epoch();
			double secondsSinceEpoch = double(duration.count()) * double(std::chrono::system_clock::period::num) / double(std::chrono::system_clock::period::den);
			std::cerr << "Shadow map received. Timestamp: " << secondsSinceEpoch << "\r\n";
			cv::imshow("Live Shadow Map", ShadowMap.Map);
			cv::waitKey(1);
		});
	}
	
	//Start the shadow detection engine, using imagery from the sim drone
	ShadowDetection::ShadowDetectionEngine::Instance().Start("Simulation"s);
	
	//Start the sim drone video feed
	myDrone->StartDJICamImageFeed(1.0);
	
	//wait until video feed is done
	while (! mySimDrone->IsSimVideoFinished())
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	//Stop the shadow detection engine and instruct it to save it's shadow map history
	ShadowDetection::ShadowDetectionEngine::Instance().Stop();
	ShadowDetection::ShadowDetectionEngine::Instance().SaveAndFlushShadowMapHistory();
	std::cerr << "Shadow map history saved to FRF files. Look in 'Shadow Map Files' folder in BIN directory.\r\n";
	
	return true;
}

static bool TestBench13(void) { return false; }
static bool TestBench14(void) { return false; }
static bool TestBench15(void) { return false; }
static bool TestBench16(void) { return false; }
static bool TestBench17(void) { return false; }
static bool TestBench18(void) { return false; }
static bool TestBench19(void) { return false; }
static bool TestBench20(void) { return false; }

//DJI Drone Interface: Simulated Drone Imagery. This test bench sets up the simulated drone for non-realtime operation and just displays video
static bool TestBench21(void) {
	DroneInterface::Drone * myDrone = DroneInterface::DroneManager::Instance().GetDrone("Simulation"s);
	if (myDrone == nullptr) {
		std::cerr << "Error: Unable to get simulated drone from drone manager.\r\n";
		return false;
	}
	DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(myDrone);
	if (mySimDrone == nullptr) {
		std::cerr << "Error: Could not down-cast Drone to SimulatedDrone.\r\n";
		return false;
	}
	mySimDrone->SetRealTime(false);
	mySimDrone->SetSourceVideoFile(Handy::Paths::ThisExecutableDirectory() / "SimSourceVideo.mov"s);
	
	//Register a callback with the drone for imagery
	myDrone->RegisterCallback([](cv::Mat const & Frame, DroneInterface::Drone::TimePoint const & Timestamp) {
		auto duration = Timestamp.time_since_epoch();
		double secondsSinceEpoch = double(duration.count()) * double(std::chrono::system_clock::period::num) / double(std::chrono::system_clock::period::den);
		std::cerr << "Image received. Timestamp: " << secondsSinceEpoch << "\r\n";
		cv::imshow("Sim Drone Imagery", Frame);
		cv::waitKey(1);
	});
	
	//Start video feed
	myDrone->StartDJICamImageFeed(1.0);
	
	//wait until video feed is done
	while (! mySimDrone->IsSimVideoFinished())
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	return true;
}

//DJI Drone Interface: Simulated Drone Imagery. This test bench sets up the simulated drone for realtime operation and just displays video
static bool TestBench22(void) {
	DroneInterface::Drone * myDrone = DroneInterface::DroneManager::Instance().GetDrone("Simulation"s);
	if (myDrone == nullptr) {
		std::cerr << "Error: Unable to get simulated drone from drone manager.\r\n";
		return false;
	}
	DroneInterface::SimulatedDrone * mySimDrone = dynamic_cast<DroneInterface::SimulatedDrone *>(myDrone);
	if (mySimDrone == nullptr) {
		std::cerr << "Error: Could not down-cast Drone to SimulatedDrone.\r\n";
		return false;
	}
	mySimDrone->SetRealTime(true);
	mySimDrone->SetSourceVideoFile(Handy::Paths::ThisExecutableDirectory() / "SimSourceVideo.mov"s);
	
	//Register a callback with the drone for imagery
	myDrone->RegisterCallback([](cv::Mat const & Frame, DroneInterface::Drone::TimePoint const & Timestamp) {
		auto duration = Timestamp.time_since_epoch();
		double secondsSinceEpoch = double(duration.count()) * double(std::chrono::system_clock::period::num) / double(std::chrono::system_clock::period::den);
		std::cerr << "Image received. Timestamp: " << secondsSinceEpoch << "\r\n";
		cv::imshow("Sim Drone Imagery", Frame);
		cv::waitKey(1);
	});
	
	//Start video feed
	myDrone->StartDJICamImageFeed(1.0);
	
	//wait until video feed is done
	while (! mySimDrone->IsSimVideoFinished())
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	return true;
}

static bool TestBench23(void) { return false; }
static bool TestBench24(void) { return false; }
static bool TestBench25(void) { return false; }




