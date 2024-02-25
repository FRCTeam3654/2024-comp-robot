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
  private double ampTimeout = 2;
  private double wristTargetPos;
  private double armTargetPos;
  private boolean isWristSmartMotionInProgress = false;


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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.intakeWheels.intakeMove(-0.2);
    //RobotContainer.intakeRollers.feedIn();
    //if(RobotContainer.wrist.isAtPos())
    if(RobotContainer.wrist.isAtPos(wristTargetPos) && RobotContainer.arm.isAtPos(armTargetPos)){
      RobotContainer.intakeRollers.feedOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.intakeWheels.intakeMove(0);
    RobotContainer.intakeRollers.stop();
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
