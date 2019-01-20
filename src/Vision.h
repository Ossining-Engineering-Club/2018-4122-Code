/*
 * This is the Vision class programmed by Nick Tremaroli
 * This was designed to JUST GET VALUES FROM GRIP
 * For any questions talk to Nick Trem
 */
#pragma once
#include <memory>
#include "WPIlib.h"
#include <algorithm>
#include <vector>
#include "Constants.h"
#define NetTable shared_ptr<NetworkTable>	// the network table uses a shared pointer, this is a smart pointer
#define Gvector vector<double>				// a vector designed for Grip

using std::vector;
using std::shared_ptr;

class Vision
{
public:
	// The contructor requires the resolution of the camera!!! --> typically 320 x 240
	Vision(unsigned int ResX, unsigned int ResY);

	// This prevents the Vision class from being copied
	Vision(Vision& src) = delete;
	Vision(Vision&& src) = delete;
	Vision& operator=(Vision& rhs) = delete;
	Vision& operator=(Vision&& rhs) = delete;

	void Update();						// Update the camera values
	void Filter();						// Experimental: DO NOT USE YET!!!!!
	void SetResolution(unsigned int x, unsigned int y);	// already required by contrustor but if changed, call this method!!!!

	int GetNumContours(); 				// returns the number of contours

	double GetArea(unsigned int val);	// returns the area
	double GetX(unsigned int val);		// returns the x value of the tracked image
	double GetY(unsigned int val);		// returns the y value of the tracked image
	double GetHeight(unsigned int val);	// returns the height
	double GetWidth(unsigned int val);	// returns the width
	double GetPercentFill(unsigned int val);	// returns the space in a % of how much space it occupies


	~Vision();					// declared but not used, add code if we start using dynamic memory allocation

private:
	NetTable Table;
	Gvector area;
	Gvector X;
	Gvector Y;
	Gvector Height;
	Gvector Width;
	unsigned int ResX;
	unsigned int ResY;
	unsigned int FullImageArea;
	bool found; 	// this is for the filter method
};
