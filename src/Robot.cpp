#include "WPIlib.h"
#include "Pneumatics.h"
#include "Tankdrive.h"
#include "Constants.h"

class Robot : public SampleRobot {
private:

	Tankdrive tankdrive;
	Pneumatics pnuematics;
	Joystick LeftStick;
	Joystick RightStick;
	Joystick LiftStick;
	Jaguar Lift;
	Talon Winch;
	Talon ArmL;
	Talon ArmR;
	Talon AngleArm;
	DigitalInput switch0;		// switch for left or right
	DigitalInput switch1;		// switch for center or not
//	DigitalInput switch2;		// switch for direct or wide
	DigitalInput switch3;		// switch for switch or scale
	DigitalInput switch4;		// switch for multi-cube
	DigitalInput switch5;
	DigitalInput LimitLift;		// switch for if we are corssing the field or not
	AnalogInput AutoPmeter;		// for the Pmeter for a starting delay during auto
	PowerDistributionPanel pdp;
	cs::UsbCamera camera;
	SmartDashboard* dash;
	bool TopLimit;

public:
	Robot():
	tankdrive(1, 0, 0, 3),
	pnuematics(0, 1),
	LeftStick(0),
	RightStick(1),
	LiftStick(2),
	Lift(2),
	Winch(3),
	ArmL(4),
	ArmR(5),
	AngleArm(6),
	switch0(5),
	switch1(6),
//	switch2(7),
	switch3(7),
	switch4(8),
	switch5(9),
	LimitLift(0),			// this is for the limit switch
	AutoPmeter(1),
	pdp(),
	camera(),
	dash()
	{
		dash->init();
		TopLimit = false;
	}

	void RobotInit()
	{
		camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(XRESOLUTION, YRESOLUTION);
		camera.SetFPS(CAMERAFPS);
		camera.SetExposureManual(CAMERAEXPOSURE);
		camera.SetBrightness(CAMERABRIGHTNESS);
		TopLimit = false;
	}

	void Autonomous()
	{
		pnuematics.compstart();
		std::string PlateColor = DriverStation::GetInstance().GetGameSpecificMessage();
		pnuematics.changeSol(0, AUTOARMSTATE);		// close the arms
		tankdrive.ResetGyro();
		tankdrive.ResetEncoders();
		dash->PutString("version", VERSION);
		dash->PutString("Message", PlateColor);
		dash->PutNumber("Gyro Angle", tankdrive.GetAngle());
		tankdrive.DirectDrive(0.0, 0.0);
		char CloseSwitch, Scale;
		CloseSwitch = PlateColor[0];
		Scale = PlateColor[1];

		Wait(AutoPmeter.GetVoltage() * 2.0);

							// just straight
		if(!switch0.Get() && !switch1.Get() && !switch3.Get() && !switch4.Get())
			tankdrive.AutoDriveGyro(AUTOSTRIAGHT, AUTOSPEED, AUTOTIMEMAX);

							// center path
		else if(!switch1.Get())
		{
			dash->PutString("Autonomous", "CENTER");
			Lift.Set(AUTOLIFTPOWER);                              //Start Lifting
//			tankdrive.AutoDriveGyro(CENTER1, AUTOSPEED, AUTOTIMEMAX);
			if(CloseSwitch == 'L')
			{
				dash->PutString("Autonomous", "CENTER LEFT");
				tankdrive.AutoTurnGyro(CLTURNANG, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				Lift.Set(AUTOLIFTCONST);
				tankdrive.AutoDriveGyro(CLDIAGDIST, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(CLTURNANGBACK, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);		// this is so the lift is definatly higher
				tankdrive.AutoDriveGyro(CENTER2L, AUTOSPEED, AUTOTIMEMICRO);
				Wait(AUTOWAIT);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
				Wait(AUTOCUBEDROPWAIT);			// have a wait for dropping the cube
				pnuematics.changeSol(0, !(AUTOARMSTATE));	// open the arms
				ArmR.Set(0.0);
				ArmL.Set(0.0);
				// check if its time for multicube
				if(!switch4.Get())
				{
					Lift.Set(AUTOLIFTDROP);
					tankdrive.AutoDriveGyro(CENTER2L, AUTOBACKUPSPEEDCRAWL, AUTOTIMEMICRO);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CRTURNANGBACK, AUTOTURNSPEEDBACK, AUTOTURNMIN);
					Wait(AUTOWAIT);
					tankdrive.AutoDriveGyro(CLDIAGDIST, AUTOBACKUPSPEED, AUTOTIMEMAX);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CRTURNANG, AUTOTURNSPEEDBACK, AUTOTURNHYPER);
					tankdrive.AutoTurnGyro(CLTURNANG, AUTOTURNSPEEDBACK, AUTOTIMESMALLEST);
					Lift.Set(0.0);
					Wait(AUTOWAIT);
					ArmR.Set(RIGHTARMSPEED);
					ArmL.Set(LEFTARMSPEED);
					tankdrive.AutoDriveGyro(AUTOWALLTOCUBE, AUTOSLOWSPEED, AUTOTIMEMAX);
					Wait(AUTOCUBEGRABWAIT);
					pnuematics.changeSol(0, AUTOARMSTATE);
					Wait(AUTOSUCKINCUBEWAIT);
					Lift.Set(AUTOLIFTLOWPOWER);
					Wait(AUTOSUCKINCUBEWAIT);
					tankdrive.AutoDriveGyro(AUTOWALLTOCUBE, AUTOBACKUPSPEEDSLOW, AUTOTIMEMAX);
					Wait(AUTOWAIT);
					ArmR.Set(0.0);
					ArmL.Set(0.0);
					Wait(0.35);

					// Now do it again!!!
					tankdrive.AutoTurnGyro(CLTURNANG2, AUTOTURNSPEED, AUTOTURNMIN);
					Wait(AUTOWAIT);
					Lift.Set(AUTOLIFTCONST);
					tankdrive.AutoDriveGyro(CLDIAGDIST, AUTOSPEED, AUTOTIMEMAX);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CLTURNANGBACK, AUTOTURNSPEED, AUTOTURNMIN);
					Wait(AUTOWAIT);		// this is so the lift is definatly higher
					tankdrive.AutoDriveGyro(CENTER2L, AUTOSPEED, AUTOTIMEMICRO);
					Wait(AUTOWAIT);
					ArmL.Set(AUTOARMSPEEDOUT);
					ArmR.Set(AUTOARMSPEEDOUT);
					Wait(AUTOCUBEDROPWAIT);			// have a wait for dropping the cube
					pnuematics.changeSol(0, !(AUTOARMSTATE));	// open the arms
					ArmR.Set(0.0);
					ArmL.Set(0.0);
					Lift.Set(AUTOLIFTCONST);

				}
			}
			else if(CloseSwitch == 'R')    // Center Right Code
			{
				dash->PutString("Autonomous", "CENTER RIGHT");
				tankdrive.AutoTurnGyro(CRTURNANG, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				Lift.Set(AUTOLIFTCONST);
				tankdrive.AutoDriveGyro(CRDIAGDIST, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(CRTURNANGBACK, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);		// this is so that the lift is higher
				tankdrive.AutoDriveGyro(CENTER2R, AUTOSPEED, AUTOTIMEMICRO);
				Wait(AUTOWAIT);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
				Wait(AUTOCUBEDROPWAIT);			// have a wait for dropping the cube
				pnuematics.changeSol(0, !(AUTOARMSTATE));	// open the arms
				ArmR.Set(0.0);
				ArmL.Set(0.0);
				if(!switch4.Get())    //Center right multi-cube after placing first Cube
				{
					Lift.Set(AUTOLIFTDROP);
					tankdrive.AutoDriveGyro(CENTER2R, AUTOBACKUPSPEEDCRAWL, AUTOTIMESHORT);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CRTURNANG, AUTOTURNSPEEDBACK, AUTOTURNMIN);
					Wait(AUTOWAIT);
					tankdrive.AutoDriveGyro(CRDIAGDIST, AUTOBACKUPSPEEDSLOW, AUTOTIMEMAX);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CLTURNANG, AUTOTURNSPEEDBACK, AUTOTURNHYPER);
					tankdrive.AutoTurnGyro(CRTURNANG, AUTOTURNSPEEDBACK, AUTOTIMESMALLEST);
					Lift.Set(0.0);
					Wait(AUTOWAIT);
					ArmR.Set(RIGHTARMSPEED);
					ArmL.Set(LEFTARMSPEED);
					tankdrive.AutoDriveGyro(AUTOWALLTOCUBE, AUTOSLOWSPEED, AUTOTIMEMID);
					Wait(AUTOCUBEGRABWAIT);
					pnuematics.changeSol(0, AUTOARMSTATE);
					Wait(AUTOSUCKINCUBEWAIT);
					Lift.Set(AUTOLIFTLOWPOWER);
					Wait(AUTOSUCKINCUBEWAIT);
					tankdrive.AutoDriveGyro(AUTOWALLTOCUBE, AUTOBACKUPSPEEDSLOW, AUTOTIMEMID);
					ArmR.Set(0.0);
					ArmL.Set(0.0);
					Wait(0.35);

					// Now do it again!!!
					tankdrive.AutoTurnGyro(CRTURNANG, AUTOTURNSPEED, AUTOTIMEMID);
					Wait(AUTOWAIT);
					Lift.Set(AUTOLIFTCONST);
					tankdrive.AutoDriveGyro(CRDIAGDIST, AUTOSPEED, AUTOTIMEMAX);
					Wait(AUTOWAIT);
					tankdrive.AutoTurnGyro(CRTURNANGBACK, AUTOTURNSPEED, AUTOTIMEMID);
					Wait(AUTOWAIT);		// this is so the lift is definatly higher
					tankdrive.AutoDriveGyro(CENTER2R, AUTOSPEED, AUTOTIMEMICRO);
					Wait(AUTOWAIT);
					ArmL.Set(AUTOARMSPEEDOUT);
					ArmR.Set(AUTOARMSPEEDOUT);
					Wait(AUTOCUBEDROPWAIT);			// have a wait for dropping the cube
					pnuematics.changeSol(0, !(AUTOARMSTATE));	// open the arms
					ArmR.Set(0.0);
					ArmL.Set(0.0);
					Lift.Set(AUTOLIFTCONST);
				}
			}
		}

							// direct right
/*		else if(!switch0.Get() && switch1.Get() && switch2.Get())
		{
			dash->PutString("Autonomous", "DIRECT RIGHT");
			Lift.Set(AUTOLIFTPOWER);
			if(CloseSwitch == 'R')
			{
				tankdrive.AutoDriveGyro(AUTODIRECTRIGHTF, AUTOSPEED, AUTOTIMEMAX);
				Lift.Set(AUTOLIFTCONST);
				Wait(AUTOWAIT);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
			}
			else
			{
				tankdrive.AutoDriveGyro(AUTODIRECTNF, AUTOSPEED, AUTOTIMEMAX);
				Lift.Set(AUTOLIFTCONST);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(TURNRIGHT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				tankdrive.AutoDriveGyro(AUTODIRECTDIAG, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(TURNLEFT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				tankdrive.AutoDriveGyro(AUTODIRECTFIN, AUTOSPEED, AUTOTIMEMAX);
			}
		}
						// direct left
		else if(switch0.Get() && switch1.Get() && switch2.Get())
		{
			dash->PutString("Autonomous", "DIRECT LEFT");
			Lift.Set(AUTOLIFTPOWER);
			if(CloseSwitch == 'L')
			{
				tankdrive.AutoDriveGyro(AUTODIRECTLEFT1, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(AUTODTURNRIGHT, AUTOTURNSPEED, AUTOTURNMIN);
				tankdrive.AutoDriveGyro(AUTODIRECTDIAG, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				Lift.Set(AUTOLIFTCONST);
				tankdrive.AutoTurnGyro(AUTODTURNLEFT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				tankdrive.AutoDriveGyro(AUTODIRECTLEFT3, AUTOSPEED, AUTOTIMEMAX);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
			}
			else
			{
				tankdrive.AutoDriveGyro(AUTODIRECTNF, AUTOSPEED, AUTOTIMEMAX);
				Lift.Set(AUTOLIFTCONST);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(TURNLEFT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				tankdrive.AutoDriveGyro(AUTODIRECTDIAG, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				tankdrive.AutoTurnGyro(TURNRIGHT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				tankdrive.AutoDriveGyro(AUTODIRECTFIN, AUTOSPEED, AUTOTIMEMAX);
			}
		}*/
		// wide right or wide left
		else if(switch1.Get())
		{
			dash->PutString("Autonomous", "WIDE");

			// DRIVE STRAIGHT --> neither switch or scale is same side as robot with no cross
			if((!switch0.Get() != (CloseSwitch == 'R')) && (!switch0.Get() != (Scale == 'R')) && switch5.Get())
			{
				dash->PutString("Autonomous", "WIDE STRIAGHT");
				tankdrive.AutoDriveGyro(AUTOSTRIAGHT, AUTOSPEED, AUTOTIMEMAX);
			}
			// CROSS THE FIELD FOR SCALE --> If the switch and scale aren't on our side but the cross switch is on
			else if((!switch0.Get() != (CloseSwitch == 'R')) && (!switch0.Get() != (Scale == 'R')) && !switch5.Get())
			{
				dash->PutString("Autonomous", "WIDE CROSSING");
				tankdrive.AutoDriveGyro(AUTOTOMIDFIELD, AUTOSPEED, AUTOTIMEMAX);
				Wait(AUTOWAIT);
				// Turn at mid field
				if(!switch0.Get())		// for right side
					tankdrive.AutoTurnGyro(TURNLEFT1, AUTOTURNSPEED, AUTOTURNMIN);
				else					// for left side
					tankdrive.AutoTurnGyro(TURNRIGHT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);
				if(!switch0.Get())		// for right side
					tankdrive.AutoDriveGyro(AUTOCROSSFIELDR, AUTOSPEED, AUTOTIMEMAX);
				else					// for left side
					tankdrive.AutoDriveGyro(AUTOCROSSFIELDL, AUTOSPEED, AUTOTIMEMAX);

				Lift.Set(AUTOLIFTPOWER);
				Wait(AUTOWAIT);
				// turn perpendicular toward scale
				if(!switch0.Get())
					tankdrive.AutoTurnGyro(TURNRIGHT1, AUTOTURNSPEED, AUTOTURNMIN);
				else
					tankdrive.AutoTurnGyro(TURNLEFT, AUTOTURNSPEED, AUTOTURNMIN);
				Wait(AUTOWAIT);

				if(!switch0.Get())			// for right side
					tankdrive.AutoDriveGyro(AUTOMIDTOSCALER, AUTOSPEED, AUTOTIMEMID);
				else
					tankdrive.AutoDriveGyro(AUTOMIDTOSCALEL, AUTOSPEED, AUTOTIMEMID);
				Wait(AUTOWAIT);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
			}
			// SWITCH PATH
			// Do if Switch is on same side as the robot and both are not true (Switch on our side, Scale Switch)
			else if((!switch0.Get() == (CloseSwitch == 'R')) && (switch3.Get() || (!switch0.Get() != (Scale == 'R'))))		// wide left
			{
				dash->PutString("Autonomous", "WIDE SWITCH");
				Lift.Set(AUTOLIFTPOWER);
				tankdrive.AutoDriveGyro(AUTOINDIRECTSWITCH, AUTOSPEED, AUTOTIMEMAX);
				Lift.Set(AUTOLIFTCONST);
				// evaluate if either left or right and then turn accordingly
				if(!switch0.Get())
					tankdrive.AutoTurnGyro(TURNLEFT, AUTOTURNSPEED, AUTOTURNMIN);
				else
					tankdrive.AutoTurnGyro(TURNRIGHT, AUTOTURNSPEED, AUTOTURNMIN);
				tankdrive.AutoDriveGyro(AUTOSWITCHLEG, AUTOLEGSPEED, AUTOTIMEMAX);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);
			}

			// SCALE PATH - Not Straight and Not switch path
			else
			{
				dash->PutString("Autonomous", "WIDE SCALE");
				Lift.Set(AUTOLIFTPOWERSCALE);
				tankdrive.AutoDriveGyro(AUTOINMID, AUTOSCALESPEED, AUTOTIMESCALEMAX, 1);

				// evaluate if either left or right and then turn accordingly
				if(!switch0.Get())
					tankdrive.AutoTurnGyro(AUTOTURNSCALEL, 0.5/*AUTOTURNSPEED*/, AUTOTURNMIN);
				else
					tankdrive.AutoTurnGyro(AUTOTURNSCALER,  0.5/*AUTOTURNSPEED*/, AUTOTURNMIN);
				Wait(AUTOWAIT);
				// maybe make the auto scale leg a little less so its not right up against the scale
				tankdrive.AutoDriveGyro(AUTODIAGTOSCALE, AUTOSLOWSPEED, AUTOTIMESCALE, 1);
				Lift.Set(AUTOLIFTCONST);
				ArmL.Set(AUTOARMSPEEDOUT);
				ArmR.Set(AUTOARMSPEEDOUT);

				// backup after we place the block
				Wait(AUTOTIMEARM);
				tankdrive.AutoDriveGyro(AUTOSCALELEG, AUTOBACKUPSPEEDSLOW, AUTOTIMESHORT);

				// check if it is time for multi-cube
				if(!switch4.Get())
				{
					pnuematics.changeSol(0, !(AUTOARMSTATE));	// open the arms
					ArmL.Set(0.0);
					ArmR.Set(0.0);
					dash->PutString("Autonomous", "WIDE SCALE MULTICUBE");
					Lift.Set(AUTOLIFTDROP);
					Wait(AUTOWAIT);
					if(!switch0.Get())
						tankdrive.AutoTurnGyroBoth(TURNLEFTSCALE1, AUTOTURNSPEEDBOTH, AUTOTURNMAX);
					else
						tankdrive.AutoTurnGyroBoth(TURNRIGHTSCALE2, AUTOTURNSPEEDBOTH, AUTOTURNMAX);

					Wait(AUTOTIMEARM);		// maybe make this wait more for the lift

					Lift.Set(0.0);
					ArmL.Set(LEFTARMSPEED);		// NEGETIVE IS IN
					ArmR.Set(RIGHTARMSPEED);


		//			dash->PutNumber("Vision exit",
		//					tankdrive.AutoDriveVision(AUTOUSCUBEDISTANCE, AUTOVISIONSPEED, AUTOTOCUBE, AUTOTIMEMID));
					tankdrive.AutoDriveGyro(AUTOTOCUBE, AUTOSLOWSPEED, AUTOTIMEMID);
					Wait(AUTOWAIT);
					ArmR.Set(0.0);
					ArmL.Set(0.0);
					pnuematics.changeSol(0, AUTOARMSTATE);	// close the arms
					Wait(AUTOCUBEGRABWAIT);

					tankdrive.AutoDriveGyro(AUTOTOCUBE, AUTOBACKUPSPEED, AUTOTIMEMID);
					if(!switch0.Get())
						tankdrive.AutoTurnGyroBoth(TURNRIGHTSCALE1, AUTOTURNSPEEDBOTH, AUTOTURNMAX);
					else
						tankdrive.AutoTurnGyroBoth(TURNLEFTSCALE2, AUTOTURNSPEEDBOTH, AUTOTURNMAX);

					tankdrive.AutoDriveGyro(AUTOSCALELEG, AUTOSLOWSPEED, AUTOTIMESHORT);
					ArmL.Set(AUTOARMSPEEDOUT);
					ArmR.Set(AUTOARMSPEEDOUT);
				}
			}
		}
		dash->PutNumber("Gyro Angle", tankdrive.GetAngle());
		tankdrive.DirectDrive(0.0, 0.0);
	}

	void OperatorControl()
	{
//		tankdrive.drive(0.0, 0.0);
		pnuematics.compstart();
		std::string PlateColor = DriverStation::GetInstance().GetGameSpecificMessage();
		unsigned short i = 0;
		tankdrive.ResetEncoders();
		while (IsOperatorControl() && IsEnabled())
		{
			for(i = 0; i < 10; i++)
			{
				tankdrive.Drive(LeftStick.GetY(), RightStick.GetY());
				tankdrive.SetThrottle((RightStick.GetZ() - 1) / 2);
			}

			if (LiftStick.GetRawButton(1))
			{
				Winch.Set(LiftStick.GetY());
				Lift.Set(-0.15);
			}																// limit is reached!!
			else if(LiftStick.GetY() < 0.1 && LiftStick.GetY() > -0.1)
			{
				Lift.Set(-0.15);
				Winch.Set(0.0);
			}
			else
			{
				if(LimitLift.Get() && LiftStick.GetY() < AUTOLIFTCONST)
					Lift.Set(AUTOLIFTCONST);
				else
					Lift.Set(LiftStick.GetY());
				Winch.Set(0.0);
			}

			if(LiftStick.GetRawButton(2))
			{
				AngleArm.Set(0.3);
			}
			else if(LiftStick.GetRawButton(3))
			{
				AngleArm.Set(-0.3);
			}
			else
				AngleArm.Set(0);

			// make sure that these convenctions are correct and that you are using talons!!!
			if(LiftStick.GetRawButton(4))
			{
				ArmR.Set(AUTOARMSPEEDOUT);			// POSITIVE IS OUT
				ArmL.Set(AUTOARMSPEEDOUT);
			}
			else if(LiftStick.GetRawButton(5))
			{
				ArmL.Set(LEFTARMSPEED);		// NEGETIVE IS IN
				ArmR.Set(RIGHTARMSPEED);
			}
			else
			{
				ArmL.Set(0.0);
				ArmR.Set(0.0);
			}

			if(LiftStick.GetRawButton(10))
			{
				pnuematics.compstart();
				dash->PutBoolean("Compressor Status", 1);
			}
			else if(LiftStick.GetRawButton(11))
			{
				pnuematics.compstop();
				dash->PutBoolean("Compressor Status", 0);
			}

			if(LiftStick.GetRawButton(6))		// check the CONVENTIONS!!!
			{
				pnuematics.changeSol(0, AUTOARMSTATE);		// closes the arms
				dash->PutBoolean("Arm Status", 0);
			}
			else if(LiftStick.GetRawButton(7))
			{
				pnuematics.changeSol(0, !(AUTOARMSTATE));		// opens the arms
				dash->PutBoolean("Arm Status", 1);
			}
			// for Displaying current vals!!!
		//	dash->PutNumber("Drive Motor #1 Current", pdp.GetCurrent(#));
		//	dash->PutNumber("Drive Motor #2 Current", pdp.GetCurrent(#));
		//	dash->PutNumber("Drive Motor #3 Current", pdp.GetCurrent(#));
		//	dash->PutNumber("Drive Motor #4 Current", pdp.GetCurrent(#));
			dash->PutNumber("Right Encoder", tankdrive.GetREncoder());
			dash->PutNumber("Left Encoder", tankdrive.GetLEncoder());
			dash->PutNumber("Gyro Angle", tankdrive.GetAngle());
			dash->PutNumber("Usonic Range", tankdrive.GetUSRange());
			dash->PutNumber("Auto Potentiometer val", AutoPmeter.GetVoltage() * 2.0);
			dash->PutNumber("switch0", !switch0.Get());
			dash->PutNumber("switch1", !switch1.Get());
			dash->PutNumber("switch3", !switch3.Get());
			dash->PutNumber("switch4", !switch4.Get());
			dash->PutNumber("switch5", !switch5.Get());
			dash->PutNumber("Limit Lift", !LimitLift.Get());
			dash->PutNumber("Lift Power", LiftStick.GetY());
			dash->PutString("version", VERSION);
			dash->PutString("Message", PlateColor);
		}

	}

};

START_ROBOT_CLASS(Robot)
