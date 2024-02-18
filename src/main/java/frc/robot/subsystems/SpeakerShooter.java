// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotMap;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;


public class SpeakerShooter extends SubsystemBase {
  /** Creates a new SpeakerShooter. */
  
  private TalonFX shooterTalonLeft  = new TalonFX (RobotMap.shooterTalonLeftID);
  private TalonFX shooterTalonRight  = new TalonFX (RobotMap.shooterTalonRightID);

  public TalonFXConfiguration shooterLeftFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration shooterRightFXConfig = new TalonFXConfiguration();
  

  public SpeakerShooter() {
    
    shooterTalonLeft.getConfigurator().apply(new TalonFXConfiguration()); //configs factory default
    shooterTalonRight.getConfigurator().apply(new TalonFXConfiguration());

    shooterTalonLeft.setControl(new VelocityDutyCycle(0)); //sets to velocity duty cycle
    shooterTalonRight.setControl(new VelocityDutyCycle(0));

    shooterTalonRight.setControl(new Follower(RobotMap.shooterTalonLeftID, true)); //sets right to follow and the true means will oppose the left's direction
    
    shooterRightFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterLeftFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Zero the sensor once on robot boot up 
		shooterTalonLeft.setPosition(0);
    shooterTalonRight.setPosition(0);

    //shooterTalonLeft.configNeutralDeadband(0.00, RobotMap.pidLoopTimeout); //during testing was 0.001
    //shooterTalonRight.configNeutralDeadband(0.00, RobotMap.pidLoopTimeout); //during testing was 0.001
    shooterRightFXConfig.MotorOutput.withDutyCycleNeutralDeadband(0);
    shooterLeftFXConfig.MotorOutput.withDutyCycleNeutralDeadband(0);
    //shooterTalonLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
    //shooterTalonRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
    shooterTalonLeft.getConfigurator().apply(new FeedbackConfigs());
    shooterTalonRight.getConfigurator().apply(new FeedbackConfigs());


    // reduce the frame rate of the follower in order to reduce the can bus utilization
    // phoenix 5 version: :  m_talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 200);
    shooterTalonRight.getPosition().setUpdateFrequency(5);  // phoenix 6 syntax
    shooterTalonRight.getVelocity().setUpdateFrequency(5);
    shooterTalonRight.getSupplyCurrent().setUpdateFrequency(5); // not sure which one is "Brushed Supply Current Measurement"
    shooterTalonRight.getStickyFault_Hardware().setUpdateFrequency(5); // so many, not sure which




    zeroSensors();
    
  }

  public void shootSpeaker(double velocity){
    //shooterTalonLeft.setControl(new VelocityDutyCycle(velocity));
  }

  
  public boolean targetSpeed(){

    
    // double speedLeft = shooterTalonLeft.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    // SmartDashboard.putNumber("Shooter Speed Top", speedTop);
    // double speedTopDifferential = speedTop - RobotMap.shooterTopSpeed_nativeUnit;
    // SmartDashboard.putNumber("Shooter Speed Top Differential", speedTopDifferential);

    // double speedRight = shooterTalonRight.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    // SmartDashboard.putNumber("Shooter Speed 2", speedBottom);
    // double speedBottomDifferential = speedBottom - RobotMap.shooterBottomSpeed_nativeUnit;
    // SmartDashboard.putNumber("Shooter Speed Differential", speedBottomDifferential);
     


     
    double speedLeft = shooterTalonLeft.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Shooter Speed Left", speedLeft);
    double speedLeftDifferential = speedLeft - RobotMap.shooterSpeed_nativeUnit;
    SmartDashboard.putNumber("Shooter Speed Top Differential", speedLeftDifferential);

    double speedRight = shooterTalonRight.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Shooter Speed 2", speedRight);
    double speedRightDifferential = speedRight - RobotMap.shooterSpeed_nativeUnit;
    SmartDashboard.putNumber("Shooter Speed Differential", speedRightDifferential);

    if ((Math.abs(speedRightDifferential) < RobotMap.shooterSpeedTolerance) && (Math.abs(speedLeftDifferential) < RobotMap.shooterSpeedTolerance)){
      return true;
    }
    else{
      return false;
    }
    

    // temp
    //return true;

  }
  

  public void shooterToVelocity(double speed) {
    shooterTalonLeft.setControl(ShooterConstants.shooterControl.withVelocity(speed));

    //shooterMotorBottom.setControl(ShooterConstants.shooterControl.withVelocity(speed));
  }

  void zeroSensors() {
    //ballShooterTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.pidLoopTimeout);
    //shooterTalonLeft.setPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    
    //shooterTalonLeft.setPosition(0);
    //shooterTalonRight.setPosition(0);

    //ballShooterBottomTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    System.out.println("Shooter Sensor is zeroed");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
