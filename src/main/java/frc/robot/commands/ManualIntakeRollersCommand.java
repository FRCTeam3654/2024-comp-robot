// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.WristSmartMotion;
import frc.robot.subsystems.IntakeRollers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualIntakeRollersCommand extends Command {
  /** Creates a new ManualIntakeRollersCommand. */
  public ManualIntakeRollersCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.oi.operatorStick.getLeftY() > 0.4){
      RobotContainer.intakeRollers.feedOut(0.1);

    }

    else if (RobotContainer.oi.operatorStick.getLeftY() < -0.4){
      RobotContainer.intakeRollers.feedIn(0.2, 0.2);

    }

    else{
      RobotContainer.intakeRollers.stop();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeRollers.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
