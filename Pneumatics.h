/*
 * Pneumatics class for FRC
 * For any question talk to Nick Tremaroli
 * MAJOR CHANGE MADE BY NICK TREM:
 *  the Pnuematics class is now based on dynamic memory allocation
 */
#pragma once
#include "WPIlib.h"

class Pneumatics
{
public: // for functions
	// The Contructor takes the Compressor port and the # of Soleniods needed for the Pneumatic system
	Pneumatics(unsigned short Compport, unsigned short NumSoleniods);

	// Prevent the Pnuematics class from being copied or double created
	Pneumatics(Pneumatics& src) = delete;
	Pneumatics(Pneumatics&& src) = delete;
	Pneumatics& operator=(Pneumatics& rhs) = delete;
	Pneumatics& operator=(Pneumatics&& rhs) = delete;

	void compstart();		// programmed to start the compressor
	void compstop();		// programmed to stop the compressor
	void changeSol(unsigned int SolNum, bool State);	// takes the index of the sol and changes the state
	~Pneumatics();		// <-- for a graceful cleanup

private: // for variables
	Compressor compressor;			// if you are using pneumatics, you need a compressor
	Solenoid *soleniods[8];			// create 8 pointers to a Solenoid
	unsigned short* NumSoleniods;	// how many soliends you want to create
};
