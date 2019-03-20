#include "Ultrasonic.h"
#include "WPILib.h"

USSensor::USSensor(unsigned int channel):
	uschannel(channel)
{
	averagerange = 0.0;
}

float USSensor::GetRawRange(void)
{
	return 8.503 * uschannel.GetVoltage();
}

void USSensor::GetSample(void)
{
	averagerange= 0.9 * averagerange + (72.0 / 15.0) * 0.1 * 8.503 * uschannel.GetVoltage();
}

float USSensor::GetRange(void)
{
	averagerange = 0.9 * averagerange + (72.0 / 15.0) * 0.1 * 8.503 * uschannel.GetVoltage();
	return averagerange;
}

