// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


/*
 *  A few possible small enhancements:
 *  1. make translationVal = 0.30;
 *  2. end the command if the intake has taken the note similar to AutoIntakeCommand's isFinished()
 *  3. estimate the distance btw robot and note in case that the targeted notes has been taken by other team
 */

public class DriveToNoteVisionTargetCommand2 extends Command {

  public static final double CHASE_NOTE_MAX_PID_OUTPUT = 0.4;
  private double driveTimer = 0;
  private double driveTimeout = 2;
  private SwerveSubsystem s_Swerve;    

  private boolean isFieldRelative;

  private boolean driveStraightFlag = false;
  private double driveStraightAngle = 0;

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(22);// same as the camera height in above ROBOT_TO_CAMERA
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-18); // camera facing down, about 18 degree from vertical line -- need double check








  public DriveToNoteVisionTargetCommand2(SwerveSubsystem s_Swerve) {
     this.s_Swerve = s_Swerve ;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);

  }

  public DriveToNoteVisionTargetCommand2( SwerveSubsystem s_Swerve, double driveTimeout) {
    this.s_Swerve = s_Swerve ;
    this.driveTimeout = driveTimeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double joystickX = 0.0;
    boolean useOpenLoop = true;

    double translationVal = 0.30;
    double strafeVal = 0.0;
    double rotationVal = 0.0;

    double distanceBtwRobotAndNote = 0.0;

    if( RobotContainer.photonLifeCam != null) {
        var results = RobotContainer.photonLifeCam.getLatestResult();
    //if( RobotContainer.photonBackOVCamera != null) {  // temp code
    //  var results = RobotContainer.photonBackOVCamera.getLatestResult();
      if( results.hasTargets() ) {
           var result = results.getBestTarget();
           if( result != null) {
                   driveStraightAngle = s_Swerve.getYawInDegree();
                   // add the vision data
                   driveStraightAngle = driveStraightAngle - result.getYaw();// add or minus need test out
                   driveStraightFlag = true;


                  distanceBtwRobotAndNote =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getPitch()));

                  SmartDashboard.putNumber("distanceBtwRobotAndNote", distanceBtwRobotAndNote);

                  // some logic here -- if the distance is long, need adjust the timeout + make sure not drive too far into other side
                  // next note in middle line is 66 inches away (~ 1.67m) ==> how long it takes to get there?
           }
      }

      // if the target is outside the vision, use the last value if driveStraight is still in progress
      if(  driveStraightFlag == true) {
               double vinniesError =  driveStraightAngle  - s_Swerve.getYawInDegree() ; 
               joystickX = vinniesError * 0.012;// 0.025;//0.01
               if(Math.abs(joystickX) > CHASE_NOTE_MAX_PID_OUTPUT) {
                joystickX = Math.signum(joystickX) * CHASE_NOTE_MAX_PID_OUTPUT;
               }

               // in drive straight mode, ignore rotation and strafe
               rotationVal = joystickX;
               strafeVal = 0;
               if( translationVal > 0.4) {
                   translationVal = 0.4; // fix the speed too?
               }
               isFieldRelative = false;
               System.out.println("Vision IP driveStraightAngle = "+driveStraightAngle+", vinniesError = "+vinniesError+", pid output ="+joystickX+", distance = "+distanceBtwRobotAndNote );
       }
    }

    s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                isFieldRelative, 
                useOpenLoop
                
            );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.intakeRollers.hasGamePiece() || (driveTimer + driveTimeout < Timer.getFPGATimestamp())  ){ 
      return true;
    }
   
    return false;
  }
}
