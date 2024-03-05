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
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToTargetPoseCommand extends Command {
  
  // should not change much, NEED match the max speed
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);//(1.5,2),(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);//(1.5, 2),(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(3, 4);//(2,2),(8, 8);
  


  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is
  private Pose2d targetPose2d = null;
  private Transform2d robotToTarget = null;


  private final ProfiledPIDController xController = new ProfiledPIDController(0.75, 0, 0, X_CONSTRAINTS); //2,(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.75, 0, 0, Y_CONSTRAINTS); //2,(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS); //0.3;// 0.5,(2, 0, 0, OMEGA_CONSTRAINTS);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  public boolean isGoalReached  = false;
  private double driveTimer;
  private double driveTimeout = 8;



  
  public AutoDriveToTargetPoseCommand(
        Pose2d targetPose2d,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.targetPose2d = targetPose2d;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(AutoConstants.autodriveXtoleranceInMeter);//0.2
    yController.setTolerance(AutoConstants.autodriveYtoleranceInMeter);//0.2
    omegaController.setTolerance(Units.degreesToRadians(AutoConstants.autodriveOmegatoleranceInDegree));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
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

    xController.setTolerance(AutoConstants.autodriveXtoleranceInMeter);//0.2
    yController.setTolerance(AutoConstants.autodriveYtoleranceInMeter);//0.2
    omegaController.setTolerance(Units.degreesToRadians(AutoConstants.autodriveOmegatoleranceInDegree));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrainSubsystem);
  }



  public AutoDriveToTargetPoseCommand(
        Transform2d robotToTarget,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.robotToTarget = robotToTarget;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(AutoConstants.autodriveXtoleranceInMeter);//0.2
    yController.setTolerance(AutoConstants.autodriveYtoleranceInMeter);//0.2
    omegaController.setTolerance(Units.degreesToRadians(AutoConstants.autodriveOmegatoleranceInDegree));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrainSubsystem);
  }

  public AutoDriveToTargetPoseCommand(
        Transform2d robotToTarget,
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
         double driveTimeout) {
          
    this.robotToTarget = robotToTarget;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.driveTimeout = driveTimeout;

    xController.setTolerance(AutoConstants.autodriveXtoleranceInMeter);//0.2
    yController.setTolerance(AutoConstants.autodriveYtoleranceInMeter);//0.2
    omegaController.setTolerance(Units.degreesToRadians(AutoConstants.autodriveOmegatoleranceInDegree));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    
  
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    isGoalReached = false;
    driveTimer = Timer.getFPGATimestamp();

    if( targetPose2d == null && robotToTarget != null) {
      // use the current robot pose and robot to target transform to get the target pose
      targetPose2d  = robotPose.transformBy(robotToTarget);
      System.out.println("targetPoseViaTransform = "+targetPose2d.toString());
      SmartDashboard.putString("targetPose = ",targetPose2d .toString());
    }

    SmartDashboard.putString("initial robot pose = ", robotPose.toString());
  }

  @Override
  public void execute() {
    
        if( targetPose2d == null) {
          return;
        }

        System.out.println("in it");
        var robotPose2d = poseProvider.get();
   
        System.out.println("Goal = "+ targetPose2d.toString());

        System.out.println("robotPose2d = "+robotPose2d.toString());

       
       // check distance and 2d angle make sense -- test code
        double distanceCamToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose2d, targetPose2d);
        System.out.println("Distance btw robot and goal = "+distanceCamToAprilTag);
       
        

        // Drive
        xController.setGoal(targetPose2d.getX());
        yController.setGoal(targetPose2d.getY());
        omegaController.setGoal(targetPose2d.getRotation().getRadians());
      
      // Drive to the target
     
      var xSpeed = xController.calculate(robotPose2d.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
        System.out.println("xController at Goal");
      }

      var ySpeed = yController.calculate(robotPose2d.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
        System.out.println("yController at Goal");
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
        System.out.println("omegaController at Goal");
      }

      // change LED color if the robot is at goal?
      
      // Get Values, Deadband
    
      xSpeed = xLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.008) );
      ySpeed = yLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.008));
      omegaSpeed =MathUtil.applyDeadband(omegaSpeed, 0.005);

      System.out.println("x,y,omega = "+xSpeed+", " +ySpeed+", "+omegaSpeed);
      // once reach goal, should not applied more power
      if( xController.atGoal() &&  Math.abs(xSpeed) < 0.001 && Math.abs(ySpeed) < 0.001 && Math.abs(omegaSpeed) < 0.005 ) {
        isGoalReached = true;
      }
      

      if( isGoalReached == true) {
        // if goal reached before, don't apply new power
        xSpeed = 0;
        ySpeed = 0;
        omegaSpeed = 0;
      }

      // this drive method is Open Loop by  boolean isOpenLoop = true; // while teleswerve now use closed loop by default now
      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation())
      );
    
    
  }

  @Override
  public void end(boolean interrupted) {
      // stop the robot?
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
