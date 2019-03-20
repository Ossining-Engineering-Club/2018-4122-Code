#pragma once
#define VERSION "SPECIAL AUTO TESTING 1.0"

// drive constants
#define LEFTMULT	.97	//was  1.05 multiplier for the left drive relative to the right

// arm constants
#define RIGHTARMSPEED	-0.2
#define LEFTARMSPEED	-0.35

// Sensot constants
#define AUTOGYROCONST 0.06			// was .07273 // was 0.016
#define ANGTOLERANCE 5.0
#define ENCODERCONST 0.07766803

// AUTONOMOUS
#define AUTOCRAWLSPEED	0.3
#define AUTOSLOWSPEED	0.45
#define AUTOSPEED		0.63		// was 0.55		// for driving WAS 0.35
#define AUTOSCALESPEED	0.8
#define AUTOBACKUPSPEEDSLOW		(AUTOSLOWSPEED * -1.0)
#define AUTOBACKUPSPEEDCRAWL	(AUTOCRAWLSPEED * -1.0)
#define AUTOBACKUPSPEED		(AUTOSPEED * -1.0)
#define AUTOTURNSPEED		0.5
#define AUTOTURNSPEEDBOTH	0.35
#define AUTOTURNSPEEDBACK	(AUTOTURNSPEED * -1.0)
#define AUTOSLOWTURNSPEED	0.2
#define AUTOSLOWTURNSPEEDBACK (AUTOSLOWTURNSPEED * -1.0)


// Autonomous times
#define AUTOTIMESMALLEST	0.45
#define AUTOTIMEMICRO		0.5
#define AUTOTIMEREGISTER	0.75
#define AUTOTIMESHORT		2.0
#define AUTOTIMEMID			3.5
#define AUTOTIMEMAX			6.5
#define AUTOTIMESCALEMAX	8.5

#define AUTOTURNMIN			1.75
#define AUTOTURNHYPER		1.0
#define AUTOTURNMID			3.0
#define AUTOTURNMAX			5.0


#define AUTOTIMEARM			0.75

// auto distances in inches
// for striaght
#define AUTOSTRIAGHT	220.0

// for center paths
#define CENTER1			10.0		// initially move 10 inches
// for center right side
#define CRTURNANG		26.09
#define CRTURNANGBACK	(CRTURNANG * -1)
#define CRDIAGDIST		83.0


// for center left side
#define CLTURNANG		-30.0
#define CLTURNANGBACK	26.0
#define CLDIAGDIST		83.0
#define CRTURNANGBACKWALL	25.0


#define CENTER2R		8.0

#define CENTER2L		6.0

// for multicube center
#define CLTURNANG2		-32.0
#define AUTOCUBEDROPWAIT	1.0
#define AUTOWALLTOCUBE		61.0
#define CRTURNBACKMULTI		-30.0

#define AUTOCUBEGRABWAIT	0.5
#define AUTOSUCKINCUBEWAIT	0.25

// for wide
#define AUTOWIDEDRIVE		136.0
#define AUTOTOSWITCH		12
#define AUTOWIDEDRIVEMORE	60.0
#define AUTOINDIRECTSWITCH  152.0
#define AUTOSWITCHLEG		25.0
#define AUTOINMID	164.0	// was 316.0
#define AUTOTIMESCALE	3.0
#define AUTOTURNSCALEL	-14.0
#define AUTOTURNSCALER (-1.0 * AUTOTURNSCALEL)
#define AUTODIAGTOSCALE		89.0

#define AUTOSCALELEG		22.0
#define AUTOLEGSPEED		0.25

// for direct right
#define AUTODIRECTRIGHTF	110.0			// full way
#define AUTODIRECTNF		50.0			// not full becuase not our color

// for direct General variables
#define AUTODIRECTDIAG		40.0			// drive diagnal
#define AUTODIRECTFIN		100.0

#define AUTODIRECTURNRIGHT	45.0
#define AUTODIRECTURNLEFT	(AUTODIRECTURNRIGHT * -1.0)

// for direct left
#define AUTODIRECTLEFT1		16.0
#define AUTODRIVELEFT2		60.0
#define AUTODIRECTLEFT3		50.0

#define AUTODTURNRIGHT		30
#define AUTODTURNLEFT		(AUTODTURNRIGHT * -1.0)

// FOR AUTO SPECIAL CASE!!!
#define AUTOTOMIDFIELD		200.0
#define AUTOCROSSFIELDL		158.0	// was 186.0
#define AUTOCROSSFIELDR		164.0	// was 186.0
#define AUTOMIDTOSCALER		18.0
#define AUTOMIDTOSCALEL		29.0
#define AUTOAPPROACHSCALE	17.0

// FOR MULTI-CUBE!!! --> double check these values again!!!
// Multi cube scale!!
#define AUTOUSCUBEDISTANCE	18.0		// stop when 18 inches from the wall
#define AUTOTOCUBE			38.0		// was 78.2

// Multi cube switch
#define AUTODIAGSWITCH		30.0

// AUTO Constants for LIFT
#define AUTOLIFTLOWPOWER	-0.55
#define AUTOLIFTPOWER		-0.85// was -0.6		// increase this value!!!
#define AUTOLIFTPOWERSCALE	-0.95		// increase this value!!!
#define AUTOLIFTCONST		-0.2	//  was -.15s
#define AUTOLIFTDROP		0.1
#define AUTOWAITLIFT		2.0		// time to wait for the lift to rise
#define AUTOARMSTATE		false
#define AUTOWAIT			0.0

// Auto Constants for ARM!!!
#define AUTOARMSPEEDOUT	0.35
#define AUTOARMSPEEDIN	(AUTOARMSPEEDOUT * -1.0)

// auto angles
#define TURNLEFT	(TURNRIGHT * -1.0)
#define TURNRIGHT	85.0

#define TURNRIGHT1	90.0
#define TURNLEFT1	(TURNRIGHT1 * -1.0)

#define TURNRIGHTSCALE2	140.0
#define TURNRIGHTSCALE1	120.0
#define TURNLEFTSCALE1	-130.0
#define TURNLEFTSCALE2	-125.0
//PID Constants
#define TIMEPERIOD 0.00001
#define PCONSTANT 0.006
#define ICONSTANT 0.025	// was 300
#define DCONSTANT 0		// was 0.000125


// VISION
#define MINVISION 168
#define MAXVISION 198
#define VISIONTARGET 178
#define VISIONMAXHEIGHT 70.0
#define XDIFFERENCEVAL 60
#define AUTODRIVEVISION 0.3

#define AUTOVISIONSPEED	0.36		// check this value!!!

// for camera values
#define XRESOLUTION			320
#define YRESOLUTION			240
#define CAMERAFPS			30
#define CAMERAEXPOSURE		35
#define CAMERABRIGHTNESS	30

// camera values
#define FPS		30
