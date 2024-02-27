// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Robot;


public class ClimbUpCommand extends Command {

  double MIN_DISTANCE  = 5; // need test this at the robot
  double MAX_DISTANCE  = 6; // need test this at the robot
  double STAY_STILL_VOLTAGE = 2.0; // need test this
  double STALL_VELOCITY = 3.0; // in RPM, need test this

  double inititalPosition = 0.0;
  double currentPosition = 0.0;

  double previousVelocity = 0.0;
  double currentVelocity = 0.0;

  boolean stopPulling = false;


  /** Creates a new ClimbUpCommand. */
  public ClimbUpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn off electric magnet
    Robot.m_pdh.setSwitchableChannel(false);
    inititalPosition = RobotContainer.climb.getSensorReading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // after certain range of distance, stop pulling up
    //currentPosition = RobotContainer.climb.getSensorReading();

    //currentVelocity = RobotContainer.climb.getSensorVelocityReading(); // in RPM for rev motor

    // need stop pulling when robot is at the max height 
    //if( Math.abs(currentPosition - inititalPosition) < MIN_DISTANCE ) {
        RobotContainer.climb.climbUp(-0.15);
    //}
    /* 
    else {
        // deceleration, currentVelocity is close to 0
        // could use other sensor value too like current SPIKE, or jerk (change in acceleration from Pigeon2)
        if( (previousVelocity - currentVelocity) > 0 && Math.abs(currentVelocity) < STALL_VELOCITY ) {
          stopPulling = true;
        }

        // just apply a static voltage to hold the robot in air
        if( stopPulling == true || Math.abs(currentPosition - inititalPosition) > MAX_DISTANCE ) {
           RobotContainer.climb.stayStillByVoltage(STAY_STILL_VOLTAGE);
        }
    }
    */
    
    //previousVelocity = currentVelocity ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climb.climbUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
