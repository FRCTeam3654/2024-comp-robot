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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;


public class IntakeRollers2 extends SubsystemBase {
        
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
    private double prevSensorReading;



    private double target1 = 0;
    private double target2 = 0;

    private ArmFeedforward m_intakeFeedforward = new ArmFeedforward(0, 0, 0);


    private double rollerTimer ;
    private double target2LagInSecond = 0.1;
    private boolean rollerStarted =  false;


    public IntakeRollers2() {
    
        
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

        //upperWheels.follow(lowerWheels);
        //lowerWheels.follow(lowerWheels);

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

        SmartDashboard.putNumber("rollerFeedInTarget1",0.4);// as default, can be modified in Shuffleboard
        SmartDashboard.putNumber("rollerFeedInTarget2",0.4);
        SmartDashboard.putNumber("rollerFeedOutTarget1",-0.4);// as default, can be modified in Shuffleboard
        SmartDashboard.putNumber("rollerFeedOutTarget2",-0.4);
        SmartDashboard.putNumber("rollerTarget2FeedInDelayInSecond",0.1);
        SmartDashboard.putNumber("rollerTarget2FeedOutDelayInSecond",0.1);

        
        // set motor to 0
  
    }

    public void feedIn() {
        //target1 = -0.5;
        //target2 = -0.5;
        target1 = SmartDashboard.getNumber("rollerFeedInTarget1",0.4); //0.4;
        target2 = SmartDashboard.getNumber("rollerFeedInTarget2",0.4); //0.4;

        //lowerWheels.set(target);
    }

    public void feedInDelayed() {
        // target1 will start at norml speed while target2 will start at minimum like 0.1, after a certain delayed period like 0.1 second, target2 back to normal speed
        //target1 = -0.5;
        //target2 = -0.5;
        target1 = SmartDashboard.getNumber("rollerFeedInTarget1",0.4); //0.4;
        target2 = 0.1;

        if( rollerStarted == false) {
            rollerTimer = Timer.getFPGATimestamp();
            rollerStarted = true;
        }
        if ( rollerStarted == true && ( (rollerTimer + SmartDashboard.getNumber("rollerTarget2FeedInDelayInSecond",0.1)) < Timer.getFPGATimestamp() ) ){ 
            target2 = SmartDashboard.getNumber("rollerFeedInTarget2",0.4); // 0.4;
        }

        //lowerWheels.set(target);
    }

    public void feedIn(double percentDuty1, double percentDuty2) {
       // target1 = percentDuty1;
       // target2 = percentDuty2;
        target1 = (-1) * percentDuty1;
        target2 = (-1) * percentDuty2;
        //prevSensorReading =  intakeNoteSensor.getAverageValue();


    }

    public void feedOut() {
        //target1 = 0.5;
        //target2 = 0.5;
        target1 = SmartDashboard.getNumber("rollerFeedOutTarget1",-0.4);//-0.4;
        target2 = SmartDashboard.getNumber("rollerFeedOutTarget2",-0.4);//-0.4;
    }

    public void feedOutDelayed() {
        //target1 = 0.5;
        //target2 = 0.5;
        target1 = SmartDashboard.getNumber("rollerFeedOutTarget1",-0.4);//-0.4;
        target2 = -0.1;//SmartDashboard.getNumber("rollerFeedOutTarget2",-0.4);//-0.4;

        if( rollerStarted == false) {
            rollerTimer = Timer.getFPGATimestamp();
            rollerStarted = true;
        }
        if ( rollerStarted == true && ( (rollerTimer + SmartDashboard.getNumber("rollerTarget2FeedOutDelayInSecond",0.1)) < Timer.getFPGATimestamp() ) ){ 
            target2 = SmartDashboard.getNumber("rollerFeedOutTarget2",-0.4); // -0.4;
        }

    }

    public void feedOut(double percentDuty) {
       // target1 = (-1.0) *  percentDuty;
        //target2 = (-1.0) *  percentDuty;

        target1 =  percentDuty;
        target2 =  percentDuty;

    }

    public void stop() {
        target1 = 0;
        target2 = 0;
    }

    public void centerNote(){
        //target1 = -0.03;
        //target2 = -0.03;

        target1 = 0.03;
        target2 = 0.03;

        double currentSensorReading = intakeNoteSensor.getAverageValue();
        if( (currentSensorReading - prevSensorReading) < 0 && currentSensorReading < 2550){
           // target1 = 0.03;
            //target2 = 0.03;

             target1 = -0.03;
            target2 = -0.03;
        }
        else if ((currentSensorReading - prevSensorReading) < 0 && currentSensorReading > 2550){
            target1 = 0;
            target2 = 0;
        }
    }

    public double getNoteSensorReading() {
        return  intakeNoteSensor.getAverageValue(); 
    }
    
    public boolean hasGamePiece(){
        SmartDashboard.putNumber("intakeSensor.getVoltage()", intakeNoteSensor.getVoltage());
        SmartDashboard.putNumber("intakeSensor.getAverageValue()", intakeNoteSensor.getAverageValue());

        if((intakeNoteSensor.getVoltage() > 1.8) || RobotContainer.oi.intakeUpButton.getAsBoolean() || intakeNoteSensor.getAverageValue() > 1600){
            prevSensorReading =  intakeNoteSensor.getAverageValue();
            return true;
        }

        prevSensorReading =  intakeNoteSensor.getAverageValue();
        //return (intakeNoteSensor.getVoltage() > 1.5);
        //return (intakeNoteSensor.getAverageValue() > 2000);
        return false;
    }

    // slightly different criteria from hasGamePiece() -- verfy the value 0.8 and 500 first
    public boolean hasNoGamePiece(){
        SmartDashboard.putNumber("intakeSensor.getVoltage()", intakeNoteSensor.getVoltage());
        SmartDashboard.putNumber("intakeSensor.getAverageValue()", intakeNoteSensor.getAverageValue());

        if((intakeNoteSensor.getVoltage() < 0.8)  && intakeNoteSensor.getAverageValue() < 500){
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      lowerWheels.set(target1);
      upperWheels.set(target2);
        //lowerWheels.set(target);
        SmartDashboard.putNumber("intakeSensor.getAverageValue()", intakeNoteSensor.getValue());
        SmartDashboard.putNumber("intakeSensor.getVoltage()", intakeNoteSensor.getVoltage());

    }
 
    public Command intakeGamepieceCommand(){
        //Command result = run(this::feedIn);
        Command result = run(this::feedIn).until(this::hasGamePiece).andThen(new WaitCommand(0.06)).andThen(this::stop);

        //Command result = run(this::feedIn).until(this::hasGamePiece).andThen(this::stop);

        return result;
    } 

    public Command centerNoteCommand(){
        Command result = run(this::centerNote);

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
