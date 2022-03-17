//module provides a class for assisting with ImGui animations
//Author: Bryan Poling
//Copyright (c) 2020 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <chrono>
#include <cmath>

//GEMS-Core Includes
#include "../TimeSeries.h"

//Variable that can smoothly transition between two values according to some transition function over time
//This is primarily useful for animating Windows and Widgets... like having them slide in/out from a program edge.
class AnimatedVariable1D {
	using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
	
	private:
		double ValA;
		double ValB;
		timeSeries1D transitionFun; //In normalized time (0 to 1) and normalized range (0 for State A, 1 for State B)
		double duration; //Actual duration of transition (in seconds)
		int state; //0: Val A, 1: A --> B transition, 2: Val B, 3: B --> A transition
		TimePoint transitionStartTime;
		
		inline double secondsIntoAnimation();
		inline void UpdateState(double SecondsIntoAnimation);
	
	public:
		AnimatedVariable1D() : ValA(0.0), ValB(0.0), state(0) { }
		inline AnimatedVariable1D(double A, double B, double Duration = 1.0); //Use default transition function
		inline AnimatedVariable1D(double A, double B, double Duration, timeSeries1D const & TransitionFun);
		~AnimatedVariable1D() { }
		
		inline void   SetA(double A) { ValA = A; }
		inline void   SetB(double B) { ValB = B; }
		
		inline void   PutInState_A() { state = 0; } //Put variable in state A now
		inline void   PutInState_B() { state = 2; } //Put variable in state B now
		inline void   Transition();                 //Transition from current state to other state (calling during transition reverses transition)
		
		//State Query
		inline double GetValue(); //Retrieve current value for variable
		inline bool   IsTransitioning(); //True if a transition is in progress
		inline bool   IsInStateA();
		inline bool   IsInStateB();
		inline bool   IsTransitioning_A_To_B();
		inline bool   IsTransitioning_B_To_A();
		
};

inline AnimatedVariable1D::AnimatedVariable1D(double A, double B, double Duration)
	: ValA(A), ValB(B), duration(Duration), state(0)
{
	//Set Dummy value for Time point - just so it is some reasonable value
	transitionStartTime = std::chrono::steady_clock::now();
	
	//Setup default transition function
	double deltaT = 0.01; //Time interval in normalized time units
	std::vector<double> times;
	std::vector<double> TransitionValues;
	TransitionValues.reserve((size_t) (1.0/deltaT) + 5U);
	for (double t = 0.0; t <= 1.0; t += deltaT) {
		double x = 0.5 - 0.5*std::cos(t * Handy::Pi<double>);
		TransitionValues.push_back(x);
	}
	transitionFun = timeSeries1D(0.0, deltaT, TransitionValues);
}

inline AnimatedVariable1D::AnimatedVariable1D(double A, double B, double Duration, timeSeries1D const & TransitionFun)
	: ValA(A), ValB(B), transitionFun(TransitionFun), duration(Duration), state(0)
{
	//Set Dummy value for Time point - just so it is some reasonable value
	transitionStartTime = std::chrono::steady_clock::now();
}

//Transition from current state to other state (calling during transition reverses transition)
inline void AnimatedVariable1D::Transition() {
	double t = secondsIntoAnimation();
	UpdateState(t);
	if (state == 0) {
		state = 1;
		transitionStartTime = std::chrono::steady_clock::now();
	}
	else if (state == 1) {
		state = 3;
		transitionStartTime = std::chrono::steady_clock::now() - std::chrono::milliseconds(uint32_t(1000.0*(duration - t)));
	}
	else if (state == 2) {
		state = 3;
		transitionStartTime = std::chrono::steady_clock::now();
	}
	else if (state == 3) {
		state = 1;
		transitionStartTime = std::chrono::steady_clock::now() - std::chrono::milliseconds(uint32_t(1000.0*(duration - t)));
	}
}

//True if transition is in progress
inline bool AnimatedVariable1D::IsTransitioning() {
	UpdateState(secondsIntoAnimation());
	return ((state == 1) || (state == 3));
}

inline bool AnimatedVariable1D::IsInStateA() {
	UpdateState(secondsIntoAnimation());
	return (state == 0);
}

inline bool AnimatedVariable1D::IsInStateB() {
	UpdateState(secondsIntoAnimation());
	return (state == 2);
}

inline bool AnimatedVariable1D::IsTransitioning_A_To_B() {
	UpdateState(secondsIntoAnimation());
	return (state == 1);
}

inline bool AnimatedVariable1D::IsTransitioning_B_To_A() {
	UpdateState(secondsIntoAnimation());
	return (state == 3);
}

//Retrieve current value for variable
inline double AnimatedVariable1D::GetValue() {
	double t = secondsIntoAnimation();
	UpdateState(t);
	if (state == 0)
		return ValA;
	else if (state == 1) {
		double tPrime = t/duration; //Normalized time
		double x2 = transitionFun(tPrime);
		double x1 = 1.0 - x2;
		return x1*ValA + x2*ValB;
	}
	else if (state == 2)
		return ValB;
	else if (state == 3) {
		double tPrime = t/duration; //Normalized time
		double x2 = transitionFun(1.0 - tPrime);
		double x1 = 1.0 - x2;
		return x1*ValA + x2*ValB;
	}
	else
		return -1.0;
}

inline double AnimatedVariable1D::secondsIntoAnimation(void) {
	TimePoint now = std::chrono::steady_clock::now();
	return (std::chrono::duration_cast<std::chrono::duration<double>>(now - transitionStartTime)).count();
}

inline void AnimatedVariable1D::UpdateState(double SecondsIntoAnimation) {
	if ((state == 1) || (state == 3)) {
		if (SecondsIntoAnimation > duration) {
			//Animation is over
			if (state == 1)
				state = 2;
			else if (state == 3)
				state = 0;
		}
	}
}





