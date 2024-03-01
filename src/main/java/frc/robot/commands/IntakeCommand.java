// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.WristSmartMotion;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */ //test edit
  //private boolean isNote = false;
  public  int intakeMode = 0;
  double intakeTimer;
  private double intakeTimeout = 30;
  public static boolean stopIntakeMotor = false;
  private double lowerRollerPercentPower = -0.5;
  private double upperRollerPercentPower = 0.3;

  public IntakeCommand(){
    //addRequirements(RobotContainer.intakeWheels);
  }
  public IntakeCommand(int intakemode) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.intakeWheels);
    this.intakeMode = intakemode;
    if( intakeMode == 2) {
      SmartDashboard.putNumber("lowerRollerPower",lowerRollerPercentPower);
      SmartDashboard.putNumber("upperRollerPower", upperRollerPercentPower);
    }
  }

   public IntakeCommand(int intakemode, int time_out) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.intakeWheels);
    this.intakeMode = intakemode;
    this.intakeTimeout = time_out;
  }

  //public static synchronized void changeMode(int new_mode){
  //  intakeMode = new_mode;
  //}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
      if (intakeMode == 0){
        // intake
        RobotContainer.intakeRollers.feedIn(-0.5,-0.5); // lower, upper
      }
      else if (intakeMode == 1){
        // spit out
        intakeTimeout = 4;
        RobotContainer.intakeRollers.feedIn(0.5,0.5);
      }
      else if (intakeMode == 2){
        // shoot to amp
        intakeTimeout = 4;
        lowerRollerPercentPower = SmartDashboard.getNumber("lowerRollerPower",-0.5);
        lowerRollerPercentPower = SmartDashboard.getNumber("upperRollerPower",0.3);
        RobotContainer.intakeRollers.feedIn(lowerRollerPercentPower, lowerRollerPercentPower);
      }
      else{
        intakeTimeout = 4;
        RobotContainer.intakeRollers.feedIn(0,0);
      }
    }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeRollers.feedIn(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if ( (intakeMode == 2 && !RobotContainer.intakeRollers.hasGamePiece())  || (intakeTimer + intakeTimeout) < Timer.getFPGATimestamp()){ //distance sensor value needs to be tuned
      return true;
    }
    return false;
  }
}
