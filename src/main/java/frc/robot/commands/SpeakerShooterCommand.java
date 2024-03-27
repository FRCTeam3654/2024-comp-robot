// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.SpeakerShooter;

import edu.wpi.first.wpilibj.Timer;

public class SpeakerShooterCommand extends Command {
  /** Creates a new SpeakerShooterCommand. */
  public double shooterTimer = 0;
  private double shooterVelocity = RobotMap.shooterVelocity;
  //for speaker: shooter speed should be -90

  public SpeakerShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.speakerShooter);
    addRequirements(RobotContainer.intakeRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.speakerShooter.shootSpeaker(shooterVelocity); // at about 95 (about 20k old value)
    //RobotContainer.speakerShooter.percentOutput(-0.9);
    RobotContainer.speakerShooter.shootSpeaker(-97); // -90, increase a little, max is about 99
    System.out.println("shoot command");
    if(RobotContainer.speakerShooter.isAtSpeed(-95)){
      //RobotContainer.intakeWheels.intakeSpin(-0.2);
      RobotContainer.intakeRollers.feedOut();
      System.out.println("should be feeding to shoot");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.speakerShooter.shootSpeaker(0);
    RobotContainer.speakerShooter.shootSpeaker(0);
    RobotContainer.intakeRollers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooterTimer + 2.5 < Timer.getFPGATimestamp()){
      return true;
    }

    return false;
  }
}
