// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class WristSmartMotion extends Command {
  /** Creates a new WristCommand. */
  public static int wristMoveNumber = 0;
  double wristTimer;
  public static boolean isSmartMotionInProgress = false;
  private boolean isDownPOVPressed = false;
  private boolean isUpPOVPressed = false;
  private boolean isSmartMotionButtonPressed = false;
  private double currentPos;
  private int mode = 0;

  public WristSmartMotion() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
  }

  public WristSmartMotion(int mode){
    addRequirements(RobotContainer.wrist);
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.oi.wristDownUpButton.getAsBoolean()  || mode == 1){
      isSmartMotionButtonPressed = true;
    }

    if(isSmartMotionButtonPressed == true){
    wristMoveNumber = wristMoveNumber + 1;

    if(wristMoveNumber %2 == 1 ){ //moves down
      wristTimer = Timer.getFPGATimestamp();
      RobotContainer.wrist.goToPositionBySmartMotion(-1 * RobotMap.wristFullUpDistance);
      //RobotContainer.wrist.setMotionMagic(0, 2000, 2000);
      System.out.println("should i be moving down");
      isSmartMotionInProgress = true;
    }

    else if(wristMoveNumber %2 != 1 ){ //moves up
      wristTimer = Timer.getFPGATimestamp();
      //RobotContainer.wrist.setMotionMagic(RobotMap.wristFullUpDistance, 2000, 2000);
      RobotContainer.wrist.goToPositionBySmartMotion(RobotMap.wristFullUpDistance); //change value depending on how much we want it to move
      System.out.println("should i be moving up");
      isSmartMotionInProgress = true;
    }
    else{
      RobotContainer.wrist.goToPositionBySmartMotion(currentPos);
      wristTimer = Timer.getFPGATimestamp();
    }
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isSmartMotionInProgress = false;
    isSmartMotionButtonPressed = false;
    isDownPOVPressed = false;
    isUpPOVPressed = false;
    mode = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (wristTimer + 1.5) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      isSmartMotionInProgress = false;
      return true;
    }
    else {
        double sensorDistance = Math.abs(RobotContainer.wrist.getSensorReading());
        double percentError = 100 * (RobotMap.wristFullUpDistance - sensorDistance)/RobotMap.wristFullUpDistance;

        if (Math.abs(percentError) < 1){
        //if (percentLeftError < 0.9 || percentLeftError < 0 )
        isSmartMotionInProgress = false;
        return true;
        }
    return false;
  }
}
}
