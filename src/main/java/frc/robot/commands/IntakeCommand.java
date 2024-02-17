// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.WristSmartMotion;
import edu.wpi.first.wpilibj.Timer;


public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  //private boolean isNote = false;
  public static int intakeMode = 0;
  double intakeTimer;
  public static boolean stopIntakeMotor = false;


  public IntakeCommand(){
    addRequirements(RobotContainer.intakeWheels);
  }
  public IntakeCommand(int intakemode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeWheels);
    this.intakeMode = intakemode;
  }

  public static synchronized void changeMode(int new_mode){
    intakeMode = new_mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.intakeWheels.noteSensor() > 1800){
      RobotContainer.intakeWheels.intakeSpin(0);
      WristSmartMotion wristSmartMotion = new WristSmartMotion(1);
      wristSmartMotion.schedule();
      WristSmartMotion.isSmartMotionInProgress = true;


    }
    else{
      if (intakeMode == 0){
        RobotContainer.intakeWheels.intakeSpin(0.2);
      }
      else if (intakeMode == 1){
        RobotContainer.intakeWheels.intakeSpin(-0.2);
      }
      else if (intakeMode == 2){
        RobotContainer.intakeWheels.intakeSpin(0);
      }
      else{
        RobotContainer.intakeWheels.intakeSpin(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.intakeWheels.intakeSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (RobotContainer.intakeWheels.noteSensor() > 1800){ //distance sensor value needs to be tuned
      //return true;
    //}
    return false;
  }
}
