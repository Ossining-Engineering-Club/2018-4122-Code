#include "Tankdrive.h"
#include "WPILib.h"
// Convencion: Teleob gets joystick vals, AUTO: feed positive vals
Tankdrive::Tankdrive(unsigned int Leftchannel, unsigned int Rightchannel, unsigned int GyroPort,
unsigned int UsonicPort):

Left(Leftchannel),
Right(Rightchannel),
LWEncoder(3,4,true,frc::Encoder::EncodingType::k4X),	// NOTE CHANGE THE ENCODER PORTS!!!!
RWEncoder(1,2,false,frc::Encoder::EncodingType::k4X),
Gyro(GyroPort),
AutoTimer(),
vision(XRESOLUTION, YRESOLUTION),
Usonic(UsonicPort)

{
	throttle = 0.0;
	VisionX = 0.0;
	LWEncoder.SetDistancePerPulse(ENCODERCONST);		//Set distance per pulse so encoders read in Inches
	RWEncoder.SetDistancePerPulse(ENCODERCONST);
	Gyro.InitGyro();
	Gyro.Reset();		// Gyro's only work on 0 and 1
}

void Tankdrive::Drive(float left, float right)
{
	// Limit left and right inputs to between -1 and 1
	if(left > 1)
		left = 1;
	else if(left < -1)
		left = -1;
	if(right > 1)
		right = 1;
	else if(right < -1)
		right = -1;
	Left.Set(left * throttle * -1);		// becuase joystick values of inversed!!!!
	Right.Set(right * throttle * -1);	// <--- ^^^^
}
void Tankdrive::DirectDrive(float left, float right)
{
	if(left > 1)
		left = 1;
	else if(left < -1)
		left = -1;
	if(right > 1)
		right = 1;
	else if(right < -1)
		right = -1;
	Left.Set(left);
	Right.Set(right);
}
void Tankdrive::SetThrottle(float Ithrottle)
{
	throttle = (1 - Ithrottle) / 2;
}

void Tankdrive::AutoDriveGyro(float distance, float speed, float TimeOut) //Args are distance, speed
{
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();

	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();

	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	while((((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		Tankdrive::DirectDrive((speed-(fabs(speed))*AUTOGYROCONST*Gyro.GetAngle()), speed+(fabs(speed))*AUTOGYROCONST*Gyro.GetAngle());
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

void Tankdrive::AutoDriveGyro(float distance, float speed, float TimeOut, bool startup)
{
	const float startfrac = 0.2;
	const int startms = 350;
	float dspeed;
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();

	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length
	int i = 0;
	while((((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		if( i>=startms )dspeed=speed;
		else dspeed = speed * (startfrac + (1-startfrac)*((float)i/(float)startms));
		Tankdrive::DirectDrive((dspeed-(fabs(dspeed))*AUTOGYROCONST*Gyro.GetAngle()), dspeed+(fabs(dspeed))*AUTOGYROCONST*Gyro.GetAngle());
		i++;
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

void Tankdrive::AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar &Lift)
{
	Lift.Set(0.0);
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;
	AutoTimer.Reset();
	AutoTimer.Start();

	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	while((((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		Tankdrive::DirectDrive((speed-AUTOGYROCONST*Gyro.GetAngle()), speed+AUTOGYROCONST*Gyro.GetAngle());
		if(!LimitLift.Get())
			Lift.Set(AUTOLIFTPOWER);
		else
			Lift.Set(AUTOLIFTCONST);
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
	Lift.Set(0.0);
}

int Tankdrive::AutoDriveVision(float USrange, float speed, float Maxdistance, float TimeOut) //Args are distance, speed
{
	int returnC = 0;
	float Sample, LastSample;  //Current Data Value and Previous data Value
	float Integral = 0.0;
	float Derivative;
	float Turn;
	LastSample = 0.0;
	for (int i = 0; i < 10; i++)
		Usonic.GetSample();
	bool USGood = 1;

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders

	if (Usonic.GetRange() < 15)
		USGood = 0;

	while(((((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get() <= TimeOut)
	{
		vision.Update();
		if (vision.GetNumContours() != 0)
		{
			if (vision.GetNumContours() == 1) VisionX = vision.GetX(0);
			else VisionX = (vision.GetX(0) + vision.GetX(1)) /2;
			Sample = VisionX - 160;
			Integral = Integral + (TIMEPERIOD/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/TIMEPERIOD;
		    // If Sample, Integral and Derivative are 0, then we want go with speed on each side
		    // If Sample, Integral or Derivative are large positive, left drive = -1, right drive = 1
		    // If Sample, Integral or Derivative are large negative, left drive = 1, right drive = -1
		    // We would like the average of the two sides to be speed

		    Turn = PCONSTANT * Sample + ICONSTANT * Integral + DCONSTANT * Derivative;
			Tankdrive::DirectDrive(speed * (1 - Turn), speed * (1 + Turn));
			LastSample = Sample;
		}
		else
			Tankdrive::DirectDrive(speed,speed); //Needed to prevent crash
		Usonic.GetSample();
		Wait(TIMEPERIOD);
	}
	if (((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) >= Maxdistance)
		returnC = 1;
	else if ((Usonic.GetRange() <= USrange ))
		returnC = 2;
	else if (AutoTimer.Get() > TimeOut)
		returnC = 3;

	Tankdrive::DirectDrive(0.0,0.0);
	return returnC;
}

void Tankdrive::AutoTurnGyroBoth(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
//	float diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle > 0.0) Tankdrive::DirectDrive(speed, -1.0 * speed);
	else Tankdrive::DirectDrive(-1.0 * speed, speed);

	while (fabs(Gyro.GetAngle()) <= fabs(angle)  && AutoTimer.Get() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{
/*		if((diff=(fabs(angle)-fabs(Gyro.GetAngle()))/ANGTOLERANCE <= 1.0))
		{
				if (angle > 0.0) Tankdrive::DirectDrive(speed * diff, -1.0 * speed * diff);
				else Tankdrive::DirectDrive(-1.0 * speed * diff, speed * diff);
		}*/
	    Wait(0.001);

	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoTurnGyro(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
//	float diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle * speed > 0.0)
		Tankdrive::DirectDrive(1.0*speed, 0.0 * speed);
	else if (angle * speed < 0.0)
		Tankdrive::DirectDrive(0.0 * speed, 1.0*speed);
	else
		return;

	while (fabs(Gyro.GetAngle()) <= fabs(angle)  && AutoTimer.Get() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{
	/*	if((diff=(fabs(angle)-fabs(Gyro.GetAngle()))/ANGTOLERANCE <= 1.0))
		{
			if (angle * speed > 0.0) Tankdrive::DirectDrive(speed * diff, 0.0);
			else Tankdrive::DirectDrive(0.0, speed * diff);
		}
			    Wait(0.001);*/
	    Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoDriveGyroUS(float USrange, float speed, float Maxdistance) //Args are distance, speed
{
	for (int i = 0; i < 10; i++)
		Usonic.GetSample();
	bool USGood=1;
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.Reset();
	RWEncoder.Reset();    //Reset Wheel Encoders
	Gyro.Reset();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	if (Usonic.GetRange() < 15) USGood=0;

	while(((((fabs(LWEncoder.GetDistance()) + fabs(RWEncoder.GetDistance())) / 2) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get()<=AUTOTIMEMAX)
	{
		Tankdrive::DirectDrive(speed-AUTOGYROCONST*Gyro.GetAngle(), speed+AUTOGYROCONST*Gyro.GetAngle());
		Usonic.GetSample();
		Wait(TIMEPERIOD);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

double Tankdrive::GetREncoder()
{
	return RWEncoder.GetDistance();
}

double Tankdrive::GetLEncoder()
{
	return LWEncoder.GetDistance();
}

void Tankdrive::ResetEncoders()
{
	RWEncoder.Reset();
	LWEncoder.Reset();
}

void Tankdrive::ResetGyro()
{
	Gyro.Reset();
}

double Tankdrive::GetAngle()
{
	return Gyro.GetAngle();
}

void Tankdrive::GetUSSample()
{
	Usonic.GetSample();
}
double Tankdrive::GetUSRange()
{
	return Usonic.GetRange();
}

