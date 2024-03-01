// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;


public class DropNoteAmpCommand extends Command {
  /** Creates a new DropNoteTrapCommand. */
  private double ampTimer = 0;
  private double ampTimeout = 30;
  private double wristTargetPos = -16.6;
  private double armTargetPos = 49;
  private boolean isWristSmartMotionInProgress = false;
  private boolean isArmSmartMotionInProgress = false;


  public DropNoteAmpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeRollers);
    addRequirements(RobotContainer.wrist);
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ampTimer = Timer.getFPGATimestamp();
    RobotContainer.wrist.goToPositionBySmartMotion(-9.5); //this should be the store pos bc the motors zero at start
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.intakeWheels.intakeMove(-0.2);
    //RobotContainer.intakeRollers.feedIn();
    if(isArmSmartMotionInProgress == false){
      //if(RobotContainer.wrist.isAtPos (-6)){
        if((ampTimer + 1) < Timer.getFPGATimestamp()){
            System.out.println("should I be going to amp");
            RobotContainer.arm.goToPositionBySmartMotion(armTargetPos);
            //System.out.println("should I be going to amp");
            isArmSmartMotionInProgress = true;
          }
    }
    
    if(RobotContainer.wrist.isAtPos(-9.5) && RobotContainer.arm.isAtPos(armTargetPos)){
      RobotContainer.intakeRollers.feedIn(-0.55, -0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.intakeWheels.intakeMove(0);
    RobotContainer.intakeRollers.stop();
    isArmSmartMotionInProgress = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!RobotContainer.intakeRollers.hasGamePiece() || (ampTimer + ampTimeout) < Timer.getFPGATimestamp()){ //distance sensor value needs to be tuned
      return true;
    }
    return false;
  }
}
