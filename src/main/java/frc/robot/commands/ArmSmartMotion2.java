// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSmartMotion2 extends Command {

  double armTimer;
  public static boolean isSmartMotionInProgress = false;
  private double intakeTimeout = 30;
  private boolean isSmartMotionButtonPressed = false;
  private double currentPos;
  private double targetPos;
  private int mode = 0;

  public ArmSmartMotion2() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  public ArmSmartMotion2(int mode){
    addRequirements(RobotContainer.arm);
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armTimer = Timer.getFPGATimestamp();
    System.out.println("ArmSmartMotion2 Command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if(mode == 0){ //storage position
        targetPos = 0;
        RobotContainer.arm.goToPositionBySmartMotion(targetPos);
        System.out.println("ARM To storage");
        isSmartMotionInProgress = true;
      }
      else if(mode == 1){ //amp position 
        targetPos = 44.5;
        RobotContainer.arm.goToPositionBySmartMotion(targetPos); //change value depending on how much we want it to move
        System.out.println("ARM TO  amp");
        isSmartMotionInProgress = true;
      }   
      else if(mode == 2){ //moves for amp
        // not used
        //targetPos = 0.5 * RobotMap.wristFullUpDistance;
        RobotContainer.arm.goToPositionBySmartMotion(targetPos); //change value depending on how much we want it to move
        System.out.println("should i be moving to amp");
        isSmartMotionInProgress = true;
      }
      else if(mode == 3){ //amp position to shoot
        //targetPos = 49;
        targetPos = SmartDashboard.getNumber("DropNoteAmpCommandArmPositionMode3",49);

        RobotContainer.arm.goToPositionBySmartMotion(targetPos); //change value depending on how much we want it to move
        // single line:  m_pidArmController.setReference(postition, CANSparkMax.ControlType.kSmartMotion);
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
        
        System.out.println("ARM To AMP "+targetPos);
        isSmartMotionInProgress = true;
      }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isSmartMotionInProgress = false;
    mode = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( (armTimer + 4) < Timer.getFPGATimestamp()) {
      // after 3 second, stop command
      isSmartMotionInProgress = false;
      return true;
    }
    else {
      double sensorDistance = Math.abs(RobotContainer.arm.getSensorReading());
      double percentError = 100;
      if( Math.abs(targetPos) > 0.01 )  {
        percentError = 100 * (targetPos - sensorDistance)/targetPos;
      }
      else {
        percentError = 10 * Math.abs(targetPos - sensorDistance); // could be 20
      }

      if (Math.abs(percentError) < 3){
        isSmartMotionInProgress = false;
        return true;
      }
        
    return false;
  }
}
}
