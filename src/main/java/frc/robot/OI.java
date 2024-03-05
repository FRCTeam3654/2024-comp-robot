/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

//import frc.robot.commands.AutonomousDriveCommand;
//import frc.robot.commands.DriveTargetCommand;
//import frc.robot.commands.BallFlushCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
//import frc.robot.commands.TurnToAngleCommand;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  //public Joystick driverStick = new Joystick(RobotMap.driverJoystickPort);
  //public Joystick operatorStick = new Joystick(RobotMap.operatorJoystickPort);

  //public CommandXboxController driverStick = new CommandXboxController(RobotMap.driverControllerPort);
  public XboxController driverStick = new XboxController(RobotMap.driverControllerPort);
  public XboxController operatorStick = new XboxController(RobotMap.operatorControllerPort);
  public Trigger turboButton;
  public JoystickButton driveStraightButton;
  public JoystickButton turnLeft90Button;
  public JoystickButton turnRight90Button; 
  public JoystickButton turnLeft180Button;
  public JoystickButton turnRight180Button;
  public JoystickButton slowSpeed;

  public JoystickButton speakerShooterButton;
  public JoystickButton intakeDownButton;
  public JoystickButton intakeUpButton;
  public JoystickButton ampButton;
  public JoystickButton ampArmButton;
  public JoystickButton climbPosButton;
      //new JoystickButton(driver, XboxController.Button.kRightBumper.value);
 

  public JoystickButton limelightButton;

  public POVButton climbUpButton;


  public Trigger sensorTrigger;

    /* Driver Buttons */
  public JoystickButton zeroGyro;
    
  public JoystickButton robotCentric;




  public OI(){

    //Driver Stick
  turboButton = new JoystickButton(driverStick, RobotMap.turboButtonNumber);
  driveStraightButton = new JoystickButton(driverStick, RobotMap.driveStraightButtonNumber);
  turnLeft90Button = new JoystickButton(driverStick, RobotMap.turnLeft90ButtonNumber);
  turnRight90Button = new JoystickButton(driverStick, RobotMap.turnRight90ButtonNumber);
  turnLeft180Button = new JoystickButton(driverStick, RobotMap.turnLeft180ButtonNumber);
  turnRight180Button = new JoystickButton(driverStick, RobotMap.turnRight180ButtonNumber);
  zeroGyro = new JoystickButton(driverStick, XboxController.Button.kY.value);
  robotCentric = new JoystickButton(driverStick, XboxController.Button.kLeftBumper.value);
  slowSpeed = new JoystickButton(driverStick, XboxController.Button.kRightBumper.value);
   



  //Operator Stick
  speakerShooterButton = new JoystickButton(operatorStick, RobotMap.speakerShooterButtonNumber);
  intakeDownButton = new JoystickButton(operatorStick, RobotMap.intakeDownButtonNumber);
  intakeUpButton = new JoystickButton(operatorStick, RobotMap.intakeUpButtonNumber);
  climbUpButton = new POVButton(operatorStick, 0);
  climbPosButton = new JoystickButton(operatorStick, RobotMap.climbPositionButtonNumber);
  ampButton = new JoystickButton(operatorStick, RobotMap.ampButtonNumber);
  ampArmButton = new JoystickButton(operatorStick, RobotMap.ampArmButtonNumber);


  

 

  //  All commands need be in RobotContainer's configureButtonBindings() now



  }
  
}
