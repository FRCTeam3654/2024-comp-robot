package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToTargetPoseCommand extends Command {
  
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is
  private Pose2d targetPose2d = null;
  private Transform2d robotToTarget = null;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
 
  public boolean isGoalReached  = false;
  private double driveTimer;
  private double driveTimeout = 4;

  
  public AutoDriveToTargetPoseCommand(
        Pose2d targetPose2d,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.targetPose2d = targetPose2d;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    addRequirements(drivetrainSubsystem);
  }

   public AutoDriveToTargetPoseCommand(
        Pose2d targetPose2d,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        double driveTimeout
        ) 
    {
          
    this.targetPose2d = targetPose2d;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.driveTimeout = driveTimeout;

    addRequirements(drivetrainSubsystem);
  }



  public AutoDriveToTargetPoseCommand(
        Transform2d robotToTarget,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          RobotContainer.led.setAll(Color.kRed);      
    this.robotToTarget = robotToTarget;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    addRequirements(drivetrainSubsystem);
  }

  public AutoDriveToTargetPoseCommand(
        Transform2d robotToTarget,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
         double driveTimeout) {
          
          RobotContainer.led.setAll(Color.kRed);      
    this.robotToTarget = robotToTarget;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.driveTimeout = driveTimeout;
    
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    
  
    var robotPose = poseProvider.get();
   
    isGoalReached = false;
    driveTimer = Timer.getFPGATimestamp();

    if(  robotToTarget != null) {
      // use the current robot pose and robot to target transform to get the target pose
      targetPose2d  = robotPose.transformBy(robotToTarget);
      //targetPose2d  = robotPose.plus(robotToTarget);
      System.out.println("targetPoseViaTransform = "+targetPose2d.toString());
      SmartDashboard.putString("targetPose = ",targetPose2d .toString());
    }

    SmartDashboard.putString("initial robot pose = ", robotPose.toString());
    System.out.println("robotPose = "+robotPose.toString());
    System.out.println("targetPose = "+targetPose2d.toString());

  }

  @Override
  public void execute() {
    
        if( targetPose2d == null) {
          return;
        }

        var robotPose2d = poseProvider.get();
   
        System.out.println("Goal = "+ targetPose2d.toString());
        System.out.println("robotPose2d = "+robotPose2d.toString());

       
       // check distance and 2d angle make sense -- test code
        double distanceRobotToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose2d, targetPose2d);
        double angleError = targetPose2d.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees();
        
     

        // Drive
       

        double xSpeed = 1.3 * (targetPose2d.getX() - robotPose2d.getX());//0.5//1.0
        double ySpeed = 1.3 * (targetPose2d.getY() - robotPose2d.getY());

        if(Math.abs(xSpeed) >ChaseTagCommand2.CHASE_TAG_MAX_PID_OUTPUT) {
          xSpeed = Math.signum(xSpeed) * ChaseTagCommand2.CHASE_TAG_MAX_PID_OUTPUT;
        }

        if(Math.abs(ySpeed) > ChaseTagCommand2.CHASE_TAG_MAX_PID_OUTPUT) {
          ySpeed = Math.signum(ySpeed) * ChaseTagCommand2.CHASE_TAG_MAX_PID_OUTPUT;
        }

        // need check the pid's output sign
        double omegaSpeed = 0.02 * angleError ;
     
    
        // change LED color if the robot is at goal?
      
        // Get Values, Deadband
    
    
        xSpeed = xLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.01) );
        ySpeed = yLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.01));
        omegaSpeed =MathUtil.applyDeadband(omegaSpeed, 0.01);

        System.out.println("x,y,omega = "+xSpeed+", " +ySpeed+", "+omegaSpeed+" with distance = "+distanceRobotToAprilTag+", angle error = "+angleError);
        // once reach goal, should not applied more power
        // either the distnace < 1 inch and angle within 2 degree,  or  very little driving power (pid output)
        if(  ( Math.abs(distanceRobotToAprilTag) < 0.03 &&  Math.abs(angleError) < 2  ) ||  ( Math.abs(xSpeed) < 0.013 && Math.abs(ySpeed) < 0.013 && Math.abs(omegaSpeed) < 0.01 ) ) {
          isGoalReached = true;
        }

        if( isGoalReached == true) {
          // if goal reached before, don't apply new power
          xSpeed = 0;
          ySpeed = 0;
          omegaSpeed = 0;
          System.out.println("Goal is reached with distance = "+distanceRobotToAprilTag+", angle error = "+angleError );
        }
        // this drive method is Open Loop by  boolean isOpenLoop = true; // while teleswerve now use closed loop by default now
        drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()), true
        );
    
    
  }

  @Override
  public void end(boolean interrupted) {
      // stop the robot?
      RobotContainer.led.setAll(Color.kGold); 
  }

  @Override
  public boolean isFinished(){
    if( isGoalReached == true) {
      return true;
    }
    else  if (  (driveTimer + driveTimeout) < Timer.getFPGATimestamp() ) { 
      return true;
    }

    return false;
  }

}
