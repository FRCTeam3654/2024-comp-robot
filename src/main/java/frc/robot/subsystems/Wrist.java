// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;


public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristSpark;
  public Wrist() {
    wristSpark = new CANSparkMax(RobotMap.wristNEOID, MotorType.kBrushless);

    wristSpark.restoreFactoryDefaults();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
