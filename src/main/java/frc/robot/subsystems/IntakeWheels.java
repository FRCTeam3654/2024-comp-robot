// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;


public class IntakeWheels extends SubsystemBase {
  /** Creates a new IntakeWheels. */
  
  private CANSparkMax upperWheels; //neo 550
  private CANSparkMax lowerWheels; //neo 550

  public double kIntakeP, kIntakeI, kIntakeD, kIntakeIz, kIntakeFF, kIntakeMaxOutput, kIntakeMinOutput;
  public double maxRPM;

  // add pid for close loop in case we need
  private SparkPIDController m_pidIntakeUpperController;
  private RelativeEncoder m_intakeUpper_encoder;

  private SparkPIDController m_pidIntakeLowerController;
  private RelativeEncoder m_intakeLower_encoder;

  private AnalogInput intakeNoteSensor;

  

  public IntakeWheels() {
    
    /* 
    upperWheels = new CANSparkMax(RobotMap.intakeNEOTopID, MotorType.kBrushless);
    lowerWheels = new CANSparkMax(RobotMap.intakeNEOBottomID, MotorType.kBrushless);

    upperWheels.restoreFactoryDefaults();
    lowerWheels.restoreFactoryDefaults();

    m_pidIntakeLowerController = lowerWheels.getPIDController();
    m_intakeLower_encoder = lowerWheels.getEncoder();

    m_pidIntakeUpperController = upperWheels.getPIDController();
    m_intakeUpper_encoder = upperWheels.getEncoder();

    //upperWheels.setInverted(true);

    kIntakeP = 0.3;  //6e-5 //make larger if it doesn't hold //0.1
    kIntakeI = 0;
    kIntakeD = 1;//0; 
    kIntakeIz = 0; 
    kIntakeFF = 0.000091;//  0.000015; //0.000156
    kIntakeMinOutput = -1;
    kIntakeMaxOutput = 1; 
    maxRPM = 11000; //5700

    // set PID coefficients
    m_pidIntakeUpperController.setP(kIntakeP);
    m_pidIntakeUpperController.setI(kIntakeI);
    m_pidIntakeUpperController.setD(kIntakeD);
    m_pidIntakeUpperController.setIZone(kIntakeIz);
    m_pidIntakeUpperController.setFF(kIntakeFF);

    m_pidIntakeLowerController.setP(kIntakeP);
    m_pidIntakeLowerController.setI(kIntakeI);
    m_pidIntakeLowerController.setD(kIntakeD);
    m_pidIntakeLowerController.setIZone(kIntakeIz);
    m_pidIntakeLowerController.setFF(kIntakeFF);

    upperWheels.follow(lowerWheels);

    upperWheels.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerWheels.setIdleMode(CANSparkMax.IdleMode.kBrake);

    kIntakeMinOutput = -0.3;
    kIntakeMaxOutput = 0.3; 
    m_pidIntakeUpperController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput); 
    m_pidIntakeLowerController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput); 

    //smart motion setting, similar to Falcon motion magic
    m_pidIntakeUpperController.setSmartMotionMaxVelocity(1500, 0); //maxVel in rpm; will need to adjust
    m_pidIntakeUpperController.setSmartMotionMaxAccel(2500,0);
    m_pidIntakeUpperController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidIntakeUpperController.setSmartMotionAllowedClosedLoopError(0.2, 0);  

    m_pidIntakeLowerController.setSmartMotionMaxVelocity(1500, 0);
    m_pidIntakeLowerController.setSmartMotionMaxAccel(2500,0);
    m_pidIntakeLowerController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidIntakeLowerController.setSmartMotionAllowedClosedLoopError(0.2, 0); 


    intakeNoteSensor = new AnalogInput(RobotMap.analogDistanceSensorPort1);
    intakeNoteSensor.setAverageBits(12);
    

    //upperWheels.setControlFramePeriodMs(40); // should we mess up control frame: rio to motor

    upperWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus1,65001); // velocity, voltage, default 20 ms
    upperWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus2,64001); // position. default 20 ms
    upperWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus4,63001); // Alternate Encoder Velocity, default 20 ms
    // no need messing with status0 (set follower's value), status3 (analog value, 50 ms)
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
  
  public void intakeMove(double speedPercent){
     /* 
    double intakeSetPoint = speedPercent*maxRPM;
    m_pidIntakeLowerController.setReference(intakeSetPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("IntakeSetPoint", intakeSetPoint);
    SmartDashboard.putNumber("IntakeVelocityVariable", m_intakeLower_encoder.getVelocity());
    */
  }

  public void intakeSpin(double percentDuty){
    //lowerWheels.set(percentDuty);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
