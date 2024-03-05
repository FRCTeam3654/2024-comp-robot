// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class TestLEDCommand extends Command {

  private double ledTimer = 0;
  private double ledTimeout = 10;
  private Color color = Color.kFuchsia;

  public TestLEDCommand(Color color) {
    this.color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.led);
  }

  public TestLEDCommand(Color color, double ledTimeout) {
    this.color = color;
    this.ledTimeout = ledTimeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.led.setAll(color);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.led.setAll(Color.kFuchsia);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( ledTimer + ledTimeout < Timer.getFPGATimestamp()){ 
      return true;
    }
    return false;
  }
}
