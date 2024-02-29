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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;


public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristMotor; //neo 1.1

  public double kWristP, kWristI, kWristD, kWristIz, kWristFF, kWristMaxOutput, kWristMinOutput;
  public double maxRPM;
  public double holdRotations;

  // add pid for close loop in case we need
  private SparkPIDController m_pidWristController;
  private RelativeEncoder m_Wrist_encoder;

  private double target;


  //private CANSparkMax wristSpark;

  public Wrist() {
     
    wristMotor = new CANSparkMax(RobotMap.wristNEOID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    m_pidWristController = wristMotor.getPIDController();
    m_Wrist_encoder = wristMotor.getEncoder();

    //upperWheels.setInverted(true);

    kWristP = 0.0005;  //6e-5 //make larger if it doesn't hold //0.1
    kWristI = 0;
    kWristD = 0;//0; 
    kWristIz = 0; 
    kWristFF = 0.000156;//  0.000015; //0.000156
    kWristMinOutput = -1;
    kWristMaxOutput = 1; 
    maxRPM = 5700; //5700

    // set PID coefficients
    m_pidWristController.setP(kWristP);
    m_pidWristController.setI(kWristI);
    m_pidWristController.setD(kWristD);
    m_pidWristController.setIZone(kWristIz);
    m_pidWristController.setFF(kWristFF);

    wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    kWristMinOutput = -0.3;
    kWristMaxOutput = 0.3; 
    m_pidWristController.setOutputRange(kWristMinOutput, kWristMaxOutput); 

    m_pidWristController.setSmartMotionMaxVelocity(1500, 0); //maxVel in rpm; will need to adjust
    m_pidWristController.setSmartMotionMaxAccel(2500,0);
    m_pidWristController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidWristController.setSmartMotionAllowedClosedLoopError(0.2, 0);  

    holdRotations = getSensorReading();

  }

  //positions for wrist: active intake, store/feed, amp shoot, trap shoot

  public void resetEncoders(){
    m_Wrist_encoder.setPosition(0);
  }

  public double getSensorReading(){
    return m_Wrist_encoder.getPosition();
  }

  public void stayStill(double rotations){
    //CANSparkMax.ControlType.kSmartMotion
    m_pidWristController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public void goHome(){
    //m_pidTurretController.setReference(0, CANSparkMax.ControlType.kPosition);
    m_pidWristController.setReference(0, CANSparkMax.ControlType.kSmartMotion);

  }

  public void goToPosition(double postition){
    m_pidWristController.setReference(postition, CANSparkMax.ControlType.kPosition);
  }
 
  public void goToPositionBySmartMotion(double postition){
    m_pidWristController.setReference(postition, CANSparkMax.ControlType.kSmartMotion);
  }


  public boolean isAtPos( double targetPos) {
    boolean isAt = false;
    if( Math.abs(targetPos) > 0.1 ) {
         if( Math.abs( (targetPos - getSensorReading()) / targetPos) < 0.05) {
           // 2 percent error
           isAt = true;
         }
        }
    else{
      //isAt = false;
      if( Math.abs( (targetPos - getSensorReading())) < 0.1) {
        return true;
      }

    }

    return isAt;
 
}

public boolean isPastPos( double targetPos) {
    boolean isPast = false;
    if( Math.abs(targetPos) > 0.1 ) {
         if( (targetPos - getSensorReading()) > 0 && targetPos < 0) {
           // 2 percent error
           isPast = true;
         }
        }
    else{
      //isAt = false;
      if( Math.abs( (targetPos - getSensorReading())) < 0.1) {
        return true;
      }

    }

    return isPast;
 
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
