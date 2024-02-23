// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;


public class IntakeRollers extends SubsystemBase {
        
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



    private double target = 0;
    private ArmFeedforward m_intakeFeedforward = new ArmFeedforward(0, 0, 0);



    public IntakeRollers() {
    
        
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
        
        // set motor to 0
  
    }

    public void feedIn() {
        target = -0.2;
    }

    public void feedIn(double percentDuty) {
        target = percentDuty;
    }

    public void feedOut() {
        target = 0.2;
    }

    public void feedOut(double percentDuty) {
        target = (1.0) *  percentDuty;
    }

    public void stop() {
        target = 0;
    }

    public boolean hasGamePiece(){
        SmartDashboard.putNumber("intakeSensor.getVoltage()", intakeNoteSensor.getVoltage());
        if(RobotContainer.oi.intakeUpButton.getAsBoolean()){
            return true;
        }
        return (intakeNoteSensor.getVoltage() > 1.5);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      
        lowerWheels.set(target);
        
    }

    public Command intakeGamepieceCommand(){
        Command result = run(this::feedIn).until(this::hasGamePiece).andThen(runOnce(this::stop));
        //Command result = run(this::feedIn).until(this::hasGamePiece);

        return result;
    } 
    
    public Command takeIn(){
        Command result = runOnce(this::feedIn);
        return result;
    } 

    public Command throwOut(){
        Command result = runOnce(this::feedOut);
        return result;
    } 

    public Command stopC(){
        Command result = runOnce(this::stop);
        return result;
    }
    
    

}
