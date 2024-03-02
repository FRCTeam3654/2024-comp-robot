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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;


public class SpeakerShooter extends SubsystemBase {
  /** Creates a new SpeakerShooter. */
  
  private TalonFX shooterTalonLeft  = new TalonFX (RobotMap.shooterTalonLeftID);
  private TalonFX shooterTalonRight  = new TalonFX (RobotMap.shooterTalonRightID);

  private VelocityVoltage m_LeftVoltage = new VelocityVoltage(0);
  //private VelocityVoltage m_RightVoltage = new VelocityVoltage(0);

  private double goalSpeedRPS = 0;

  public TalonFXConfiguration shooterLeftFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration shooterRightFXConfig = new TalonFXConfiguration();
  

  public SpeakerShooter() {
    
    // https://github.com/FRC-5013-Park-Hill-Robotics/2024-Crescendo/blob/main/src/main/java/frc/robot/constants/LauncherConstants.java
    shooterLeftFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterLeftFXConfig.Slot0.kP = 0.05; // 0.0254;
    shooterLeftFXConfig.Slot0.kI = 0;
    shooterLeftFXConfig.Slot0.kD = 0;
    shooterLeftFXConfig.Slot0.kS = 0.15;//0.395;
    shooterLeftFXConfig.Slot0.kV = 0.12;//0.122;
    shooterLeftFXConfig.Slot0.kA = 0.0;
    shooterLeftFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterLeftFXConfig.CurrentLimits.StatorCurrentLimit = 50;
    shooterLeftFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    shooterLeftFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    shooterTalonLeft.set(0);
    shooterTalonLeft.getConfigurator().apply(shooterLeftFXConfig); 
    m_LeftVoltage.withSlot(0);

    shooterRightFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterRightFXConfig.Slot0.kP = 0.05; //0.0254;
    shooterRightFXConfig.Slot0.kI = 0;
    shooterRightFXConfig.Slot0.kD = 0;
    shooterRightFXConfig.Slot0.kS = 0.15;
    shooterRightFXConfig.Slot0.kV = 0.12;
    shooterRightFXConfig.Slot0.kA = 0.0;
    shooterRightFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterLeftFXConfig.CurrentLimits.StatorCurrentLimit = 50;
    shooterLeftFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterLeftFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    shooterLeftFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    shooterTalonRight.set(0);
    shooterTalonRight.getConfigurator().apply(shooterRightFXConfig);
    m_LeftVoltage.withSlot(0);

    
    //shooterTalonLeft.setControl(new VelocityDutyCycle(0)); //sets to velocity duty cycle
    //shooterTalonRight.setControl(new VelocityDutyCycle(0));

    shooterTalonLeft.setControl(new VelocityVoltage(0));
    shooterTalonRight.setControl(new VelocityVoltage(0));


    shooterTalonRight.setControl(new Follower(RobotMap.shooterTalonLeftID, true)); //sets right to follow and the true means will oppose the left's direction
    

    
    // Zero the sensor once on robot boot up 
		shooterTalonLeft.setPosition(0);
    shooterTalonRight.setPosition(0);

    

    // reduce the frame rate of the follower in order to reduce the can bus utilization
    // phoenix 5 version: :  m_talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 200);
    // ref: https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html 
    //      https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/status-signals-guide.html
    // Note: When different update frequencies are specified for signals that share a status frame, the highest update frequency of all the relevant signals will be applied to the entire frame. Users can get a signal’s applied update frequency using the getAppliedUpdateFrequency() method.
    //shooterTalonRight.getPosition().setUpdateFrequency(5);  // phoenix 6 syntax
    //shooterTalonRight.getVelocity().setUpdateFrequency(5);
    //shooterTalonRight.getSupplyCurrent().setUpdateFrequency(5); // not sure which one is "Brushed Supply Current Measurement"
    //shooterTalonRight.getStickyFault_Hardware().setUpdateFrequency(5); // so many, not sure which

    //shooterTalonRight.getFault_Hardware().setUpdateFrequency(20);// group 1, default 10 ms, set to 20 ms Motor controllers that are followers can have slower update rates for this group without impacting performance.
    //shooterTalonRight.getForwardLimit().setUpdateFrequency(20);


    
  }

  public void shootSpeaker(double velocity){
    //shooterTalonLeft.setControl(new VelocityDutyCycle(velocity));
    this.goalSpeedRPS = velocity; // in rps
    if(goalSpeedRPS == 0){
      shooterTalonLeft.setVoltage(0);
    } else {
      m_LeftVoltage.withVelocity(goalSpeedRPS);
      shooterTalonLeft.setControl(m_LeftVoltage);
    }
    
  }





  public void percentOutput(double percentDuty){
    System.out.println("percent 0.3");
    shooterTalonLeft.setControl(new DutyCycleOut(percentDuty));
  }

  
  public double getShooterVelocity() {
     return shooterTalonLeft.getVelocity().getValueAsDouble();
  }
  
  public boolean isAtSpeed( double targetSpeed) {
     boolean isAt = false;
     System.out.println("shooter at speed" + shooterTalonLeft.getVelocity().getValueAsDouble());
     if( Math.abs(targetSpeed) > 0.1 ) {
          if( Math.abs( (targetSpeed - shooterTalonLeft.getVelocity().getValueAsDouble()) / targetSpeed) < 0.20) {
            // 2 percent error
            isAt = true;
          }
     }
     else {
        if( Math.abs(targetSpeed - shooterTalonLeft.getVelocity().getValueAsDouble()) < 0.5 ) {
           isAt = true;
        }
     }

     return isAt;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }
}
