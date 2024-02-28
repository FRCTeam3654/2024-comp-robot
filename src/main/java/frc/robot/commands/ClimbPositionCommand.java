// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;


public class ClimbPositionCommand extends Command {
  /** Creates a new DropNoteTrapCommand. */
  private double climbPosTimer = 0;
  private double climbPosTimeout = 30;
  private double wristTargetPos = -28;
  private double armTargetPos = 33;
  private boolean isWristSmartMotionInProgress = false;
  private boolean isArmSmartMotionInProgress = false;


  public ClimbPositionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wrist);
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbPosTimer = Timer.getFPGATimestamp();
    RobotContainer.wrist.goToPositionBySmartMotion(wristTargetPos); //this should be the store pos bc the motors zero at start
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.intakeWheels.intakeMove(-0.2);
    //RobotContainer.intakeRollers.feedIn();
    if(isArmSmartMotionInProgress == false){
      if(RobotContainer.wrist.isAtPos(-12)){
            RobotContainer.arm.goToPositionBySmartMotion(armTargetPos);
            isArmSmartMotionInProgress = true;
          }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.intakeWheels.intakeMove(0);
    isArmSmartMotionInProgress = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((climbPosTimer + climbPosTimeout) < Timer.getFPGATimestamp()){
      return true;
    }
    return false;
  }
}
