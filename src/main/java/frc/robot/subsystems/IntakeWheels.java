// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;


public class IntakeWheels extends SubsystemBase {
  /** Creates a new IntakeWheels. */
  private CANSparkMax upperWheels;
  private CANSparkMax lowerWheels;


  private AnalogInput intakeNoteSensor;

  public IntakeWheels() {
    /* 
    upperWheels = new CANSparkMax(RobotMap.intakeNEOTopID, MotorType.kBrushless);
    lowerWheels = new CANSparkMax(RobotMap.intakeNEOBottomID, MotorType.kBrushless);

    upperWheels.restoreFactoryDefaults();
    lowerWheels.restoreFactoryDefaults();

    //upperWheels.setInverted(true);

    upperWheels.follow(lowerWheels);

    upperWheels.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerWheels.setIdleMode(CANSparkMax.IdleMode.kBrake);


    intakeNoteSensor = new AnalogInput(RobotMap.analogDistanceSensorPort1);
    intakeNoteSensor.setAverageBits(12);
    */
  }

   
  public double noteSensor() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    /* 
    double cmDistanceSensor1;
    cmDistanceSensor1 = intakeNoteSensor.getAverageValue();
    SmartDashboard.putNumber("note sensor raw", cmDistanceSensor1);
    return cmDistanceSensor1;
    */
    return 0; // temp code
  }
  
  public void intakeMove(double speed){
     // lowerWheels.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
