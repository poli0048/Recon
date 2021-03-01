//This module provides encapsulation for 1D, 2D, 3D, and 4D time series objects. These are typedeffed for convenience (timeSeries1D, timeSeries2D, etc.)
//After creation, a time series can be prepared for thread-safe access by calling the finalize() method. Do not assume access to a time series is
//thread-safe until this is called (even for reading values).
//Author: Bryan Poling
//Copyright (c) 2015 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <fstream>
#include <type_traits>
#include <vector>
#include <list>
#include <iomanip>

//Project Includes
#include "EigenAliases.h"

//In order to work with Eigen types, we internally use the Eigen aligned memory allocator for STL containers of values. This is true,
//regardless of whether or not the value type of the timeSeries is an Eigen type. Constructors are offered using regular STL containers as well as for
//Eigen-specialized STL containers. The constructors using regular STL containers are not compatible with Vector-valued time series.
template <typename valueType> class timeSeries {
	public:
		//Ensure at compile time that the timeSeries class is only templated with an approved value type. Supports double, Vector2d, Vector3d, and Vector4d.
		static_assert(std::is_same<valueType,double>::value          ||
		              std::is_same<valueType,Eigen::Vector2d>::value ||
		              std::is_same<valueType,Eigen::Vector3d>::value ||
		              std::is_same<valueType,Eigen::Vector4d>::value,
		              "timeSeries is only templatable with double and Vector2/3/4d");
		
		//Constructors and destructors
		timeSeries() { }
		timeSeries(std::vector<double> const & Times, std::vector<valueType> const & Values);  //Build non-uniform time series from time and value vectors
		timeSeries(std::vector<double> const & Times, std::Evector<valueType> const & Values); //Build non-uniform time series from time and value vectors
		timeSeries(double T0, double deltaT, std::vector<valueType> const & Values);           //Build uniform time series from value vector
		timeSeries(double T0, double deltaT, std::Evector<valueType> const & Values);          //Build uniform time series from value vector
		~timeSeries() { }
		
		//Inspection methods and modifiers
		bool         empty(void);     //Returns true if the objects contains no data - false otherwise
		void         clear(void);     //Clear all internal data (as if object was freshly constructed)
		unsigned int size(void);      //Returns number of entries in time series
		double       startTime(void); //Returns time of first element in time series, or 0 is series is empty
		double       endTime(void);   //Returns time of last element in time series, or 0 is series is empty
		double       duration(void);  //Returns (endTime - startTime)
		void         finalize(void);  //Prepares the time series for thread-safe data retrieval
		
		//Data access methods
		valueType operator()(double time); //Interpolate or extrapolate the time series and get the value at the provided instant (0 if object is empty)
		valueType front(void);             //Get first element (0 if the timeseries is empty)
		valueType back(void);              //Get last element (0 if the timeseries is empty)
		double    getTimeByIndex (unsigned int index); //Get the time at the given index (0 if there is no value for the provided index)
		valueType getValueByIndex(unsigned int index); //Get the value at the given index (0 if there is no value for the provided index)
		
	private:
		std::Evector<valueType> values; //STL vector of values with Eigen memory allocator (for aligned mallocs that work with Eigen)
		
		bool uniformTimeSeries = false; //True if all measurements are exactly evenly spaced in time. False otherwise
		
		//If the time series is uniform, we can quickly get an index from a time by knowing the time of the first measurement and the deltaT
		double uniformTimeSeriesT0 = 0.0;
		double uniformTimeSeriesDeltaT = 0.0;
		
		//If the time series is not uniform, we use a mapping vector to get an index that is close to the desired time and then we search around to get
		//the right index. timeStampToIndexVector is used to store this mapping. To get an index close to the time t:
		//Let x = (unsigned int) (timeMultiplier * (t - times[0]))
		//the value y = timeStampToIndexVector[x] is an index so that times[y] is close to t.
		std::vector<double> times;
		double timeMultiplier = 0.0;
		std::vector<int> timeStampToIndexVector;
		unsigned int getLeadingIndexFromTimestamp(double time);
		void prepareObjectForNonUniformReading(void);
		unsigned int sanitizeIndex(unsigned int Index);
		
		//Linearly interpolate between two time-series elements
		static valueType linearInterpolation(double t1, valueType const & Y1, double t2, valueType const & Y2, double t);
		
		//Private methods for rendering value items to a stream
		static void printItem(std::ostream & os, double const & x);
		static void printItem(std::ostream & os, Eigen::Vector2d const & x);
		static void printItem(std::ostream & os, Eigen::Vector3d const & x);
		static void printItem(std::ostream & os, Eigen::Vector4d const & x);
		
		//Private method to grab a 0-initialized value object - this method is implemented through template specialization
		static valueType getZero(void) { throw; }
		
		static valueType coordinateWiseMin(valueType const & V1, valueType const & V2) { throw; } //Implemented through template specialization
		static valueType coordinateWiseMax(valueType const & V1, valueType const & V2) { throw; } //Implemented through template specialization
};

//************************************************************************************************************************************************************************
//*********************************************************************   Template specialiations   **********************************************************************
//************************************************************************************************************************************************************************
template <> inline double          timeSeries<double>::getZero(void)          { return(0.0); }
template <> inline Eigen::Vector2d timeSeries<Eigen::Vector2d>::getZero(void) { return(Eigen::Vector2d(0.0, 0.0)); }
template <> inline Eigen::Vector3d timeSeries<Eigen::Vector3d>::getZero(void) { return(Eigen::Vector3d(0.0, 0.0, 0.0)); }
template <> inline Eigen::Vector4d timeSeries<Eigen::Vector4d>::getZero(void) { return(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)); }

template <> inline double          timeSeries<double>::coordinateWiseMin(double const & V1, double const & V2) { return std::min(V1, V2); }
template <> inline Eigen::Vector2d timeSeries<Eigen::Vector2d>::coordinateWiseMin(Eigen::Vector2d const & V1, Eigen::Vector2d const & V2) {
	return Eigen::Vector2d(std::min(V1(0), V2(0)), std::min(V1(1), V2(1)));
}
template <> inline Eigen::Vector3d timeSeries<Eigen::Vector3d>::coordinateWiseMin(Eigen::Vector3d const & V1, Eigen::Vector3d const & V2) {
	return Eigen::Vector3d(std::min(V1(0), V2(0)), std::min(V1(1), V2(1)), std::min(V1(2), V2(2)));
}
template <> inline Eigen::Vector4d timeSeries<Eigen::Vector4d>::coordinateWiseMin(Eigen::Vector4d const & V1, Eigen::Vector4d const & V2) {
	return Eigen::Vector4d(std::min(V1(0), V2(0)), std::min(V1(1), V2(1)), std::min(V1(2), V2(2)), std::min(V1(3), V2(3)));
}

template <> inline double          timeSeries<double>::coordinateWiseMax(double const & V1, double const & V2) { return std::max(V1, V2); }
template <> inline Eigen::Vector2d timeSeries<Eigen::Vector2d>::coordinateWiseMax(Eigen::Vector2d const & V1, Eigen::Vector2d const & V2) {
	return Eigen::Vector2d(std::max(V1(0), V2(0)), std::max(V1(1), V2(1)));
}
template <> inline Eigen::Vector3d timeSeries<Eigen::Vector3d>::coordinateWiseMax(Eigen::Vector3d const & V1, Eigen::Vector3d const & V2) {
	return Eigen::Vector3d(std::max(V1(0), V2(0)), std::max(V1(1), V2(1)), std::max(V1(2), V2(2)));
}
template <> inline Eigen::Vector4d timeSeries<Eigen::Vector4d>::coordinateWiseMax(Eigen::Vector4d const & V1, Eigen::Vector4d const & V2) {
	return Eigen::Vector4d(std::max(V1(0), V2(0)), std::max(V1(1), V2(1)), std::max(V1(2), V2(2)), std::max(V1(3), V2(3)));
}

//************************************************************************************************************************************************************************
//***************************************************************************   Constructors   ***************************************************************************
//************************************************************************************************************************************************************************
//Build non-uniform time series from time and value vectors. Times and Values *must* have the same number of elements. Times should contain strictly monotonically increasing
//values - for performance, this is not checked internally. If there is any risk that the times may be non-monotonic, check and fix this before using this constructor.
template <typename valueType> timeSeries<valueType>::timeSeries(std::vector<double> const & Times, std::vector<valueType> const & Values) {
	//Ensure at compile time that this constructor is only being used on time series of compatible types.
	static_assert(std::is_same<valueType,double>::value,
	              "Constructor using standard STL containers are not compatible with Vector-valued time series.");
	clear();
	values.reserve(Values.size());
	if (Times.size() != Values.size()) {
		fprintf(stderr,"Error: Cannot create time series from value and time lists of different sizes.\r\n");
		return;
	}
	uniformTimeSeries = false;
	values.insert(values.begin(), Values.begin(), Values.end());
	times = Times;
}

//Build non-uniform time series from time and value vectors. Times and Values *must* have the same number of elements. Times should contain strictly monotonically increasing
//values - for performance, this is not checked internally. If there is any risk that the times may be non-monotonic, check and fix this before using this constructor.
template <typename valueType> timeSeries<valueType>::timeSeries(std::vector<double> const & Times, std::Evector<valueType> const & Values) {
	clear();
	if (Times.size() != Values.size()) {
		fprintf(stderr,"Error: Cannot create time series from value and time lists of different sizes.\r\n");
		return;
	}
	uniformTimeSeries = false;
	values = Values;
	times = Times;
}

//Build uniform time series from value vector
template <typename valueType> timeSeries<valueType>::timeSeries(double T0, double deltaT, std::vector<valueType> const & Values) {
	//Ensure at compile time that this constructor is only being used on time series of compatible types.
	static_assert(std::is_same<valueType,double>::value,
	              "Constructor using standard STL containers are not compatible with Vector-valued time series.");
	clear();
	values.reserve(Values.size());
	if (deltaT < 1e-9) {
		fprintf(stderr,"Error: Cannot create time series with such a small deltaT (%f)\r\n", deltaT);
		return;
	}
	uniformTimeSeries = true;
	uniformTimeSeriesT0 = T0;
	uniformTimeSeriesDeltaT = deltaT;
	values.insert(values.begin(), Values.begin(), Values.end());
}

//Build uniform time series from value vector
template <typename valueType> timeSeries<valueType>::timeSeries(double T0, double deltaT, std::Evector<valueType> const & Values) {
	clear();
	if (deltaT < 1e-9) {
		fprintf(stderr,"Error: Cannot create time series with such a small deltaT (%f)\r\n", deltaT);
		return;
	}
	uniformTimeSeries = true;
	uniformTimeSeriesT0 = T0;
	uniformTimeSeriesDeltaT = deltaT;
	values = Values;
}

//************************************************************************************************************************************************************************
//*****************************************************************   Inspection methods and modifiers   *****************************************************************
//************************************************************************************************************************************************************************
//Clear all internal data (as if object was freshly constructed)
template <typename valueType> void timeSeries<valueType>::clear(void) {
	values.clear();
	times.clear();
	timeStampToIndexVector.clear();
	
	uniformTimeSeries = false;
	uniformTimeSeriesT0 = 0.0;
	uniformTimeSeriesDeltaT = 0.0;
	timeMultiplier = 0.0;
}

//Returns number of entries in time series
template <typename valueType> unsigned int timeSeries<valueType>::size(void) {
	return((unsigned int) values.size());
}

//Returns true if the object contains no data - false otherwise
template <typename valueType> bool timeSeries<valueType>::empty(void) {
	return(values.empty());
}

//Returns time of first element in time series, or 0 if series is empty
template <typename valueType> double timeSeries<valueType>::startTime(void) {
	if (values.empty()) return(0);
	if (uniformTimeSeries) return(uniformTimeSeriesT0);
	else return(times.front());
}

//Returns time of last element in time series, or 0 if series is empty
template <typename valueType> double timeSeries<valueType>::endTime(void) {
	if (values.empty()) return(0);
	if (uniformTimeSeries) return(uniformTimeSeriesT0 + uniformTimeSeriesDeltaT*(((double) values.size()) - 1.0));
	else return(times.back());
}

//Returns (endTime - startTime)
template <typename valueType> double timeSeries<valueType>::duration(void) {
	return(endTime() - startTime());
}

//Prepares the time series for thread-safe data retrieval
template <typename valueType> void timeSeries<valueType>::finalize(void) {
	if ((! uniformTimeSeries) && (timeStampToIndexVector.empty()))
		prepareObjectForNonUniformReading();
}

//************************************************************************************************************************************************************************
//***********************************************************************   Data access methods   ************************************************************************
//************************************************************************************************************************************************************************
//Interpolate or extrapolate the time series and get the value at the provided instant (0 if object is empty)
template <typename valueType> valueType timeSeries<valueType>::operator()(double time) {
	if (values.empty()) return(getZero());
	
	double leadingTime      = 0.0;
	valueType leadingValue  = getZero();
	double trailingTime     = 0.0;
	valueType trailingValue = getZero();
	if (uniformTimeSeries) {
		double t0 = uniformTimeSeriesT0;
		double tf = uniformTimeSeriesT0 + uniformTimeSeriesDeltaT*(((double) values.size()) - 1.0);
		if (time <= t0) return(values.front());
		if (time >= tf) return(values.back());
		unsigned int leadingIndex  = (unsigned int) floor((time - uniformTimeSeriesT0)/uniformTimeSeriesDeltaT);
		unsigned int trailingIndex = leadingIndex + 1U;
		if (leadingIndex  > ((unsigned int) values.size()) - 1U) leadingIndex  = ((unsigned int) values.size()) - 1U;
		if (trailingIndex > ((unsigned int) values.size()) - 1U) trailingIndex = ((unsigned int) values.size()) - 1U;
		leadingTime   = uniformTimeSeriesT0 + (((double) leadingIndex) * uniformTimeSeriesDeltaT);
		leadingValue  = values[leadingIndex];
		trailingTime  = uniformTimeSeriesT0 + (((double) trailingIndex) * uniformTimeSeriesDeltaT);
		trailingValue = values[trailingIndex];
	}
	else {
		//Check to see if our timestamp to index vector is populated - if not, then populate it
		if (timeStampToIndexVector.empty()) prepareObjectForNonUniformReading();
		double t0 = times.front();
		double tf = times.back();
		if (time <= t0) return(values.front());
		if (time >= tf) return(values.back());
		unsigned int leadingIndex = getLeadingIndexFromTimestamp(time);
		unsigned int trailingIndex = leadingIndex + 1U;
		if (leadingIndex  > ((unsigned int) values.size()) - 1U) leadingIndex  = ((unsigned int) values.size()) - 1U;
		if (trailingIndex > ((unsigned int) values.size()) - 1U) trailingIndex = ((unsigned int) values.size()) - 1U;
		leadingTime   = times[leadingIndex];
		leadingValue  = values[leadingIndex];
		trailingTime  = times[trailingIndex];
		trailingValue = values[trailingIndex];
	}
	
	//Interpolate between the leading and trailing values
	return(linearInterpolation(leadingTime, leadingValue, trailingTime, trailingValue, time));
}

//Get first element
template <typename valueType> valueType timeSeries<valueType>::front(void) {
	if (values.empty()) return(getZero());
	else return(values.front());
}

//Get last element
template <typename valueType> valueType timeSeries<valueType>::back(void) {
	if (values.empty()) return(getZero());
	else return(values.back());
}

//Get the time at the given index (0 if there is no value for the provided index)
template <typename valueType> double timeSeries<valueType>::getTimeByIndex(unsigned int index) {
	if (index >= (unsigned int) values.size()) return(0.0);
	double time = 0.0;
	if (uniformTimeSeries)
		time = uniformTimeSeriesT0 + (((double) index) * uniformTimeSeriesDeltaT);
	else
		time = times[index];
	return(time);
}

//Get the value at the given index (0 if there is no value for the provided index)
template <typename valueType> valueType timeSeries<valueType>::getValueByIndex(unsigned int index) {
	//std::cerr << "Test 1\r\n";
	//valueType defaultValue(0.0);
	//std::cerr << defaultValue << "\r\n";
	if (index >= (unsigned int) values.size()) return(getZero());
	else return(values[index]);
}


//************************************************************************************************************************************************************************
//*******************************************************************   Private (Non-Static) Methods   *******************************************************************
//************************************************************************************************************************************************************************
//Return the highest index corresponding to a time before "time". If there are no measurements with a time before "time", return 0.
template <typename valueType> unsigned int timeSeries<valueType>::getLeadingIndexFromTimestamp(double time) {
	if (times.empty()) return(0U);
	
	int index = (int) (timeMultiplier*(time - times[0]));
	if (index < 0) index = 0;
	if (index > ((int) timeStampToIndexVector.size()) - 1) index = ((int) timeStampToIndexVector.size()) - 1;
	index = timeStampToIndexVector[index];
	if (index < 0) index = 0;
	if (index > (int) (times.size() - 1)) index = (int) (times.size() - 1);
	
	//Scan backwards until we find an index corresponding to a time before "time"
	while ((index >= 0) && (times[index] >= time)) index--;
	if (index < 0) return(0U);
	
	//Scan forwards until we find the first index corresponding to a time at or after "time"
	while ((index < (int) times.size()) && (times[index] < time)) index++;
	if (index >= (int) times.size()) return(((unsigned int) times.size()) - 1U);
	
	return(index - 1); //Return index just before the first measurement with time greater than or equal to "time"
}

//This method is to be called lazily upon the first attempted read from a non-uniform time series. This ensures that the indexing overhead is skipped for non-uniform time series
//which are never read from.
template <typename valueType> void timeSeries<valueType>::prepareObjectForNonUniformReading(void) {
	//This is the target number of timestamps that will generally get mapped to the same index. The smaller this number is, the faster
	//index lookup will be, but at the expense of greater memory consumption. There is no additional speed to be gained by setting this < 2.0
	double timestampIndexQuantization = 2.0;
	
	timeMultiplier = 0.0;
	timeStampToIndexVector.clear();
	if (values.empty()) {
		fprintf(stderr,"Trying to prepare an empty time series for non-uniform reading. Nothing to do.\r\n");
		return;
	}
	
	double SPS = ((double) times.size())/(times.back() - times.front());
	timeMultiplier = SPS/timestampIndexQuantization;
	int maxIndex = (int) (timeMultiplier*(times.back() - times.front()));
	timeStampToIndexVector.resize(maxIndex+1, -1);
	for (int i=0; i<(int) times.size(); i++) {
		int index = (int) floor(timeMultiplier*(times[i] - times.front()));
		if (index < 0) index = 0;
		if (index > maxIndex) index = maxIndex;
		timeStampToIndexVector[index] = i;
	}
	int lastIndex = 0;
	for (int i=0; i<=maxIndex; i++) {
		if (timeStampToIndexVector[i] == -1)
			timeStampToIndexVector[i] = lastIndex;
		else
			lastIndex = timeStampToIndexVector[i];
	}
}

template <typename valueType> unsigned int timeSeries<valueType>::sanitizeIndex(unsigned int Index) {
	if (values.empty())
		return 0U;
	else
		return std::min(Index, ((unsigned int) values.size()) - 1U);
}

//************************************************************************************************************************************************************************
//*********************************************************************   Private (Static) Methods   *********************************************************************
//************************************************************************************************************************************************************************
template <typename valueType> valueType timeSeries<valueType>::linearInterpolation(double t1, valueType const & Y1, double t2, valueType const & Y2, double t) {
	if (t2 - t1 < 0.0001) t2 = t1 + 0.0001; //Stabilize division by ensuring denominators are not too small. Favor Y1 in this event
	double dt = t2 - t1;
	return((t - t1)/dt*Y2 + (t2 - t)/dt*Y1);
}

template <typename valueType> void timeSeries<valueType>::printItem(std::ostream & os, double const & x) {
	os << std::setw(26) << std::setprecision(16) << x;
}

template <typename valueType> void timeSeries<valueType>::printItem(std::ostream & os, Eigen::Vector2d const & x) {
	os << std::setw(26) << std::setprecision(16) << x(0) << ", "
	   << std::setw(26) << std::setprecision(16) << x(1);
}

template <typename valueType> void timeSeries<valueType>::printItem(std::ostream & os, Eigen::Vector3d const & x) {
	os << std::setw(26) << std::setprecision(16) << x(0) << ", "
	   << std::setw(26) << std::setprecision(16) << x(1) << ", "
	   << std::setw(26) << std::setprecision(16) << x(2);
}

template <typename valueType> void timeSeries<valueType>::printItem(std::ostream & os, Eigen::Vector4d const & x) {
	os << std::setw(26) << std::setprecision(16) << x(0) << ", "
	   << std::setw(26) << std::setprecision(16) << x(1) << ", "
	   << std::setw(26) << std::setprecision(16) << x(2) << ", "
	   << std::setw(26) << std::setprecision(16) << x(3);
}

//************************************************************************************************************************************************************************
//*****************************************************************************   Typedefs   *****************************************************************************
//************************************************************************************************************************************************************************
typedef timeSeries<double>          timeSeries1D;
typedef timeSeries<Eigen::Vector2d> timeSeries2D;
typedef timeSeries<Eigen::Vector3d> timeSeries3D;
typedef timeSeries<Eigen::Vector4d> timeSeries4D;



