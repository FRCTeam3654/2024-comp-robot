// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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


public class Arm extends SubsystemBase {
  /** Creates a new Wrist. */
  /* */
  private CANSparkMax armMotor; //neo 1.1

  public double kArmP, kArmI, kArmD, kArmIz, kArmFF, kArmMaxOutput, kArmMinOutput;
  public double maxRPM;

  // add pid for close loop in case we need
  private SparkPIDController m_pidArmController;
  private RelativeEncoder m_Arm_encoder;

  private double target = 0;
 

 
  private CANSparkMax armSpark;
  public Arm() {
    /* 
    armSpark = new CANSparkMax(RobotMap.wristNEOID, MotorType.kBrushless);

    armSpark.restoreFactoryDefaults();

    m_pidArmController = armMotor.getPIDController();
    m_Arm_encoder = armMotor.getEncoder();

    //upperWheels.setInverted(true);

    kArmP = 0.3;  //6e-5 //make larger if it doesn't hold //0.1
    kArmI = 0;
    kArmD = 1;//0; 
    kArmIz = 0; 
    kArmFF = 0.000156;//  0.000015; //0.000156
    kArmMinOutput = -1;
    kArmMaxOutput = 1; 
    maxRPM = 5700; //5700

    // set PID coefficients
    m_pidArmController.setP(kArmP);
    m_pidArmController.setI(kArmI);
    m_pidArmController.setD(kArmD);
    m_pidArmController.setIZone(kArmIz);
    m_pidArmController.setFF(kArmFF);

    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    kArmMinOutput = -0.3;
    kArmMaxOutput = 0.3; 
    m_pidArmController.setOutputRange(kArmMinOutput, kArmMaxOutput); 

    m_pidArmController.setSmartMotionMaxVelocity(1500, 0); //maxVel in rpm; will need to adjust
    m_pidArmController.setSmartMotionMaxAccel(2500,0);
    m_pidArmController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidArmController.setSmartMotionAllowedClosedLoopError(0.2, 0);  

    */

  }

  //positions for wrist: active intake, store/feed, amp shoot, trap shoot

  public void resetEncoders(){
    //m_Arm_encoder.setPosition(0);
  }

  //public double getSensorReading(){
   // return m_Arm_encoder.getPosition();
  //}
/* 
  public void stayStill(double rotations){
    //CANSparkMax.ControlType.kSmartMotion
    m_pidArmController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public void goHome(){
    //m_pidTurretController.setReference(0, CANSparkMax.ControlType.kPosition);
    m_pidArmController.setReference(0, CANSparkMax.ControlType.kSmartMotion);

  }

  public void goToPosition(double postition){
    m_pidArmController.setReference(postition, CANSparkMax.ControlType.kPosition);
  }
 
  public void goToPositionBySmartMotion(double postition){
    m_pidArmController.setReference(postition, CANSparkMax.ControlType.kSmartMotion);
  }

  
    public void amp() {
        target = 15;
        m_pidArmController.setReference(target, CANSparkMax.ControlType.kSmartMotion);
    }

    public void amp(double smartMotionPos) {
        target = smartMotionPos;
        m_pidArmController.setReference(smartMotionPos, CANSparkMax.ControlType.kSmartMotion);

    }

    public void store() {
        target = 0;
        m_pidArmController.setReference(target, CANSparkMax.ControlType.kSmartMotion);

    }

    public void store(double smartMotionPos) {
        target = smartMotionPos;
        m_pidArmController.setReference(smartMotionPos, CANSparkMax.ControlType.kSmartMotion);

    }

    public void trap() {
        target = 15;
        m_pidArmController.setReference(target, CANSparkMax.ControlType.kSmartMotion);
    }

    public void trap(double smartMotionPos) {
        target = smartMotionPos;
        m_pidArmController.setReference(smartMotionPos, CANSparkMax.ControlType.kSmartMotion);

    }

    public void stop() {
        target = 0;
    }
    */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
    //armSpark.set(target);
  }
/* 
     public Command armAmpCommand(){
        //Command result = run(this::feedIn).until(this::hasGamePiece).andThen(runOnce(this::stop));
        Command result = run(this::amp);

        return result;
    } 
    
    public Command armStoreCommand(){
        Command result = runOnce(this::store);
        return result;
    } 

    public Command armTrapCommand(){
        Command result = runOnce(this::trap);
        return result;
    } 

    public Command stopC(){
        Command result = runOnce(this::stop);
        return result;
    }
    */
  }

