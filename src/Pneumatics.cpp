#include "Pneumatics.h"

Pneumatics::Pneumatics(unsigned short Compport, unsigned short NumSoleniods)
: compressor(0)
{
	this->NumSoleniods = new unsigned short;
	*(this->NumSoleniods) = NumSoleniods;
	for(int i = 0; i < *(this->NumSoleniods); i++)
		soleniods[i] = new Solenoid(i);

}

void Pneumatics::compstart()
{
	compressor.Start();
}

void Pneumatics::compstop()
{
	compressor.Stop();
}

void Pneumatics::changeSol(unsigned int SolNum, bool State)
{
	if(SolNum < *NumSoleniods)			// don't go outside of the bounds
		(soleniods[SolNum])->Set(State);
}

Pneumatics::~Pneumatics()
{
	// Time to CLEAN UP!!!!
	for(int i = 0; i < *NumSoleniods; i++)
	{
		delete(soleniods[i]);
		soleniods[i] = nullptr;
	}

	delete(NumSoleniods);
	NumSoleniods = nullptr;
}
