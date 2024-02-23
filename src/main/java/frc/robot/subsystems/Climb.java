// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.OI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;


public class Climb extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax climbMotor1;
  private CANSparkMax climbMotor2; //neo 1.1

  public double kClimbP, kClimbI, kClimbD, kClimbIz, kClimbFF, kclimbMaxOutput, kClimbMinOutput;
  public double maxRPM;

  // add pid for close loop in case we need
  private SparkPIDController m_pidClimb1Controller;
  private SparkPIDController m_pidClimb2Controller;
  private RelativeEncoder m_Climb1_encoder;
  private RelativeEncoder m_Climb2_encoder;


  private CANSparkMax climbSpark1;
  private CANSparkMax climbSpark2;


  public Climb() {
    climbSpark1 = new CANSparkMax(RobotMap.wristNEOID, MotorType.kBrushless);
    climbSpark2 = new CANSparkMax(RobotMap.wristNEOID, MotorType.kBrushless);

    climbSpark1.restoreFactoryDefaults();
    climbSpark2.restoreFactoryDefaults();

    m_pidClimb1Controller = climbMotor1.getPIDController();
    m_pidClimb2Controller = climbMotor2.getPIDController();

    m_Climb1_encoder = climbMotor1.getEncoder();  
    m_Climb2_encoder = climbMotor2.getEncoder();

    //upperWheels.setInverted(true);

    kClimbP = 0.3;  //6e-5 //make larger if it doesn't hold //0.1
    kClimbI = 0;
    kClimbD = 1;//0; 
    kClimbIz = 0; 
    kClimbFF = 0.000156;//  0.000015; //0.000156
    kClimbMinOutput = -1;
    kclimbMaxOutput = 1; 
    maxRPM = 5700; //5700

    // set PID coefficients
    m_pidClimb1Controller.setP(kClimbP);
    m_pidClimb1Controller.setI(kClimbI);
    m_pidClimb1Controller.setD(kClimbD);
    m_pidClimb1Controller.setIZone(kClimbIz);
    m_pidClimb1Controller.setFF(kClimbFF);

    m_pidClimb2Controller.setP(kClimbP);
    m_pidClimb2Controller.setI(kClimbI);
    m_pidClimb2Controller.setD(kClimbD);
    m_pidClimb2Controller.setIZone(kClimbIz);
    m_pidClimb2Controller.setFF(kClimbFF);

    climbSpark1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climbSpark2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    kClimbMinOutput = -0.3;
    kclimbMaxOutput = 0.3; 
    m_pidClimb1Controller.setOutputRange(kClimbMinOutput, kclimbMaxOutput); 
    m_pidClimb2Controller.setOutputRange(kClimbMinOutput, kclimbMaxOutput);

    m_pidClimb1Controller.setSmartMotionMaxVelocity(1500, 0); //maxVel in rpm; will need to adjust
    m_pidClimb1Controller.setSmartMotionMaxAccel(2500,0);
    m_pidClimb1Controller.setSmartMotionMinOutputVelocity(0, 0);
    m_pidClimb1Controller.setSmartMotionAllowedClosedLoopError(0.2, 0);  

    m_pidClimb2Controller.setSmartMotionMaxVelocity(1500, 0); //maxVel in rpm; will need to adjust
    m_pidClimb2Controller.setSmartMotionMaxAccel(2500,0);
    m_pidClimb2Controller.setSmartMotionMinOutputVelocity(0, 0);
    m_pidClimb2Controller.setSmartMotionAllowedClosedLoopError(0.2, 0);  

    climbMotor2.follow(climbMotor1);

  }

  //positions for wrist: active intake, store/feed, amp shoot, trap shoot

  public void resetEncoders(){
    m_Climb1_encoder.setPosition(0);
  }

  public double getSensorReading(){
    return m_Climb1_encoder.getPosition();
  }

  public void stayStill(double rotations){
    //CANSparkMax.ControlType.kSmartMotion
    m_pidClimb1Controller.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public void goHome(){
    //m_pidTurretController.setReference(0, CANSparkMax.ControlType.kPosition);
    m_pidClimb1Controller.setReference(0, CANSparkMax.ControlType.kSmartMotion);

  }

  public void goToPosition(double postition){
    m_pidClimb1Controller.setReference(postition, CANSparkMax.ControlType.kPosition);
  }
 
  public void goToPositionBySmartMotion(double postition){
    m_pidClimb1Controller.setReference(postition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void climbUp(double speed){
    double setPoint = speed * maxRPM;
    m_pidClimb1Controller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
