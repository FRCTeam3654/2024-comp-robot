/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// imports 
package frc.robot;
import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorMatch;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

static Preferences prefs; 

//DRIVE VALUES!
//test branch
public static double driveStraightProportion = 0.01;//0.02
public static double turnDegreeProportion = 0.005;
public static double LimelightJoeyX = -0.007;
public static double turn90DegreeTimeout = 3;
public static double limeLightTimeout = 2;
public static double driveToAprilTagProportion = 0.005;//0.02
public static double driveToPhotonvisionProportion = 0.005;//0.02
public static double driveToPhotonvisionByPositionProportion = 2.5;


//public static double ballStorageTimerAndysVision = 2;
public static double autonomousVelocity = 0.5; 
public static double leftOverRightCompensation = .98;
public static double talonDriveAccelerationRate = 0.1654;//0.3654
//public static double ballStorageSpeed = -0.6;
public static double effectiveTurnWheelWidth = 0.64; // meter, measured by turning the robot

// ^^^ Must be experimentally derived

public static double driveDeadband = 0.05;
public static double autonomousTargetPos = 219340.8/2; //gear ratio is :  10.71:1  219340.8 is ten rounds of big wheel
public final static double kMeterToFalconSenorUnit = 45812.56; // measured effective wheel diameter is 6.0 inches
public final static int kSensorUnitsPerRotation = 2048;
public final static double kNeutralDeadband = 0.001; // testing MP

public static double leftPercentOutput = 0.9;
public static double rightPercentOutput = 0.9;
public static double nonTurboMultiplierTurn = 0.2; //0.15
public static double nonTurboMultiplierForward = 0.72; //.5, .40 0.72

public static boolean driveClosedLoopMode = true;

//BUTTON/PORT NUMBERS!
public static int driverControllerPort = 0;
public static int operatorControllerPort = 1;

//DRIVE STICK
public static int turboButtonNumber = 18;
public static int driveStraightButtonNumber = 7; //want to be right trigger
public static int turnLeft90ButtonNumber = 3; //blue x
public static int turnRight90ButtonNumber = 2; //red b //might only do 1 180 button so i have an extra
public static int turnLeft180ButtonNumber = 4; //yellow y
public static int turnRight180ButtonNumber = 1; //green a
public static int aprilButtonNumber = 5; //left bumper
public static int noteSensorButtonNumber = 6; //right bumper




//OPERATOR STICK
public static int speakerShooterButtonNumber = 4; //yellow y
public static int ampButtonNumber = 1; // green a
public static int intakeDownButtonNumber = 3; // blue x
public static int intakeUpButtonNumber = 2; //red b
public static int trapButtonNumber = 6; //right bumper
public static int climbPositionButtonNumber = 5; //left bumper //this button is for raising the arm to hook
//public static int climbWinchButtonNumber = 18; //want to be right trigger //this is for moving the winch



//TALONS/TALON ID NUMBERS!

public static int analogDistanceSensorPort1 = 1; 
public static int digitalDistanceSensorPort2 = 0; 
public static int digitalDistanceSensorPort3 = 3; 
//public static int digitalDistanceSensorPort4 = 3; 
//public static int digitalDistanceSensorPort5 = 4; 

//talon motor IDs
public static int shooterTalonLeftID = 5;
public static int shooterTalonRightID = 10;

//non-talon motor IDs
public static int intakeNEOTopID = 7;
public static int intakeNEOBottomID = 8;

public static int climbMotor1ID = 30;
public static int climbMotor2ID = 40;

public static int wristNEOID = 9;

public static int armNEOID = 25;

public static double joystickDeadBand = 0.08;

public static double autonomousTimeOut = 40; // used to be 7 second in normal auto mode but 2021 is different

public static double autonomousBallShooterTimeOut = 4;
//public static double autonomousBallPickUpTimeOut = 25; // MICHELE WAS HERE

public static double motionMagicTimeOut = 4;// in regular, it should time out in 4 seconds


//Values for random things in the code (not swerve drive related)
public static double shooterVelocity = 0.5;
public static double shooterSpeed_nativeUnit = 50; //need to configure the correct rpm (phoenix 6 uses rotations per second)
public static double shooterSpeedTolerance = 10; //need to configure to the correct rpm

public static double PIDLoopTimeout = 5;

//falcon maximum velocity in native unit 
public static int maximumVelocityFalcon = 21000; // need to change for new robot
public static double radianConversionToDegree = 57.2958;

public static final int LED_PWMPORT_LEFT = 9;
public static final int LED_PWMPORT_RIGHT = 8;
public static final int LED_LENGTH = 18;

//For the Wrist
public static double wristFullUpDistance = 2; //will tune later

//For the Arm
public static double armAmpDistance = 0.5;

/**
 * which PID slot to pull gains from. starting 2018, you can choose from 0,1,2 or 3.
 * only the first two (0,1) are visible in web-based
 * configuration
 */
public static final int kSlotIDx = 0; //default for drive
//public static final int kColorWheelSlotIDx = 1;
//public static final int kPickUpArmSlotIDx = 1;
public static final int kClimbSlotIDx = 2;
public static final int kShooterSlotIDx = 3;
public static final int kTurnAutonomousSlotIDx = 1;
public static final boolean kUseMotionProfileArc = false;

public final static int PID_PRIMARY = 0;
public final static int PID_TURN = 1;
public final static int REMOTE_0 = 0;
public final static int REMOTE_1 = 1;

public static boolean isOpenLoop = true;//false;

/*these are the pid gains responsiveness to the control loop
*kF: 1023 represents toutput value to Talon at 100%, 7200 represents velocity units at 100% output
*                                                         kP,  kI,  kD,    kF,  Iz,  PeakOutput*/
//public final static Gains driveGainsVelocity = new Gains( 0.25, 0.0, 0.0, 1.015, 400, 1);
//public final static Gains driveGainsVelocity = new Gains( 0.095, 0.0, 0.0, 0.0451, 100, 1);

//static final Gains kGains = new Gains(0.03, 0.0, 0, 0.0451, 0, 0.5);
public static int pidLoopTimeout = 30;

/**
 * Talon SRX/ Victor SPX will supported miltiple (cascaded) PID Loops For 
 * now we just want the primary one.
 */
public static final int kPIDLoopIDx = 0;
//public static final int pickUpArmPIDLoopIDx = 0;
/**
 * set to zero to skip waiting for confirmation, set to nonzero to wait
 *  wait and report to DS if action fails
 */
public static final int kTimeoutMs = 30;
//public static final int pickUpArmTimeoutMs = 30;
/** --GAINs--
 * gains used to Motion Magic, to be adjusted acccordingly
 * Gains(kp, ki, kd, kf, izone, peak output);
 */
//public static final Gains pickUpArmGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
//public static final Gains kGains = new Gains(0.095, 0.0, 0, 0.0451, 0, 0.25); // colorWheelTalon  use this



}

