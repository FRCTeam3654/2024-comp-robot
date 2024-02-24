// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class ArmSmartMotion extends Command {
  /** Creates a new ArmSmartMotion. */
  //public static int wristMoveNumber = 0;
  double armTimer;
  public static boolean isSmartMotionInProgress = false;
  //private boolean isDownPOVPressed = false;
  //private boolean isUpPOVPressed = false;
  private boolean isSmartMotionButtonPressed = false;
  private double currentPos;
  private double targetPos;
  private int mode = 0;

  public ArmSmartMotion() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  public ArmSmartMotion(int mode){
    addRequirements(RobotContainer.arm);
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   if(RobotContainer.oi.intakeDownButton.getAsBoolean() || RobotContainer.oi.intakeUpButton.getAsBoolean() || mode == 1){
    isSmartMotionButtonPressed = true;
   }

    if(isSmartMotionButtonPressed == true){
    //wristMoveNumber = wristMoveNumber + 1;

    if(mode == 0){ //moves full down for intake
      armTimer = Timer.getFPGATimestamp();
      targetPos = RobotMap.armAmpDistance;
      //RobotContainer.arm.goToPositionBySmartMotion(targetPos);

      System.out.println("should i be moving down");
      isSmartMotionInProgress = true;
      //IntakeCommand.changeMode(0);
    }

    else if(mode == 1){ //moves to storage position
      armTimer = Timer.getFPGATimestamp();
   
      targetPos = 0;
     // RobotContainer.arm.goToPositionBySmartMotion(targetPos); //change value depending on how much we want it to move
      System.out.println("should i be moving to storage");
      isSmartMotionInProgress = true;
      //IntakeCommand.changeMode(2);

    }
    
    else if(mode == 2){ //moves for amp
      armTimer = Timer.getFPGATimestamp();

      targetPos = 0.5 * RobotMap.wristFullUpDistance;
      //RobotContainer.wrist.goToPositionBySmartMotion(targetPos); //change value depending on how much we want it to move
      System.out.println("should i be moving to amp");
      isSmartMotionInProgress = true;
    }

    else{
      //currentPos = RobotContainer.arm.getSensorReading();
      //RobotContainer.arm.goToPositionBySmartMotion(currentPos);
      armTimer = Timer.getFPGATimestamp();
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
    //isDownPOVPressed = false;
    //isUpPOVPressed = false;
    mode = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (armTimer + 3) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      isSmartMotionInProgress = false;
      return true;
    }
    else {
      double sensorDistance = Math.abs(RobotContainer.arm.getSensorReading());
      double percentError = 100;
      if( Math.abs(targetPos) > 0.001 )  {
        percentError = 100 * (targetPos - sensorDistance)/targetPos;
      }
      else {
        percentError = 10 * Math.abs(targetPos - sensorDistance); // could be 20
      }

      if (Math.abs(percentError) < 1){
        isSmartMotionInProgress = false;
        return true;
      }
        
    return false;
  }
}
}
