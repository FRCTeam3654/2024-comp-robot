package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand2 extends Command {
   
  // front AprilTag Camera -- roll or pitch , which is 30 degree?
  
  public static final double CHASE_TAG_MAX_SPEED = 1.5; // swerve can have max speed 4 or higher, but is not good for chasing tag
  public static final double CHASE_TAG_MAX_PID_OUTPUT = 0.4;

  public static final Transform3d ROBOT_TO_CAMERA_FRONT = new Transform3d(
        new Translation3d(Units.inchesToMeters(-7), Units.inchesToMeters(3.5), Units.inchesToMeters(21)),
        new Rotation3d(0, Units.degreesToRadians(0), 0)); 

  // back camera's pitch: is it -30 degree since it is looking up , ( not 150 since yaw turns 180 degree, pitch stays 30 degree looking up) : decouple pitch from yaw
  public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(20)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))); 

 
  // drive backward,use back camera to align to speaker
  private static final Transform3d TAG_TO_GOAL_SPEAKER = 
      new Transform3d(
          new Translation3d(1.0, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0));

  // AMP's TAG_TO_GOAL
  private static final Transform3d TAG_TO_GOAL_AMP = 
      new Transform3d(
          new Translation3d(Units.inchesToMeters(18), 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0));
  
  // Stage's TAG_TO_GOAL
  private static final Transform3d TAG_TO_GOAL_STAGE = 
      new Transform3d(
          new Translation3d(Units.inchesToMeters(33), 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

          
  private Transform3d which_tag_to_goal;

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is

 

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  
  public boolean isGoalReached  = false;

  private PhotonTrackedTarget lastTarget;
  private Pose3d robotPose3dByVision = null;
  private Pose2d  goalPose;
  private double distanceCamToAprilTag;

  private double tagTimer ;
  private double tagTimeout = 20;

  public ChaseTagCommand2(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
 
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    
    lastTarget = null;
    var robotPose = poseProvider.get();

   
    isGoalReached = false;

    tagTimer = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    
    var robotPose2d = poseProvider.get();
    // robot is in 2 dimensional (X,Y)
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .7 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {


        var target = targetOpt.get();//PhotonTrackedTarget
        if( photonRes.getBestTarget() != null) {
          target = photonRes.getBestTarget();
        }

        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        int fiducialId = target.getFiducialId();

        Pose3d aprilTagPose3d = RobotContainer.poseEstimator.aprilTagFieldLayout.getTagPose(fiducialId).get();
        System.out.println("aprilTagPose3d = "+aprilTagPose3d.toString());
        
       
        //////////////////////////////////////////////////////////////////////////////////
        
        if (target.getPoseAmbiguity() <= .7 && fiducialId >= 0   && aprilTagPose3d != null   ) {
          Transform3d camToTarget = target.getBestCameraToTarget();
          

          Pose3d camPose3d = aprilTagPose3d.transformBy(camToTarget.inverse());
          // check distance and 2d angle make sense

          // which camera, front or back
          robotPose3dByVision = camPose3d.transformBy(ROBOT_TO_CAMERA_BACK.inverse());//(CAMERA_TO_ROBOT);

          robotPose = robotPose3dByVision; // trust the vision more here than poseEstimator 
        }

        ///////////////////////////////////////////////////////////////////////////////////


        if(fiducialId == 4 || fiducialId == 7 ) {
            // speaker
            which_tag_to_goal = TAG_TO_GOAL_SPEAKER;
        }
        else if(fiducialId == 5 || fiducialId == 6 ) {
            // AMP
            which_tag_to_goal = TAG_TO_GOAL_AMP;
        }
        else  if( fiducialId >= 11) {
            // All stages
            which_tag_to_goal = TAG_TO_GOAL_STAGE;
        }
        else {
            which_tag_to_goal = null;
        }

        if( which_tag_to_goal != null ) {
            // Transform the tag's pose to set our goal
            goalPose = aprilTagPose3d.transformBy(which_tag_to_goal).toPose2d();

            System.out.println("Goal = "+ goalPose.toString());

            System.out.println("robotPose2d = "+robotPose2d.toString());

            if( robotPose3dByVision != null) {
                // check distance and 2d angle make sense -- test code
                distanceCamToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose3dByVision.toPose2d(), goalPose);
                System.out.println("Distance btw robot and goal = "+distanceCamToAprilTag);    
            }  
        }
      }
    }
  
    if (lastTarget == null || which_tag_to_goal == null) {
      // No target has been visible
      //drivetrainSubsystem.stop();
    } else {
      // Drive to the target
    

      double xSpeed = 0.5 * (goalPose.getX() - robotPose.getX());
      double ySpeed = 0.5 * (goalPose.getY() - robotPose.getY());

      if(Math.abs(xSpeed) > CHASE_TAG_MAX_PID_OUTPUT) {
        xSpeed = Math.signum(xSpeed) * CHASE_TAG_MAX_PID_OUTPUT;
      }

      if(Math.abs(ySpeed) > CHASE_TAG_MAX_PID_OUTPUT) {
        ySpeed = Math.signum(ySpeed) * CHASE_TAG_MAX_PID_OUTPUT;
      }


      

      // need check the pid's output sign
      var omegaSpeed = 0.02 * (goalPose.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees()) ;
      
      xSpeed = xLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.01) );
      ySpeed = yLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.01));
      //omegaSpeed = omegaLimiter.calculate(  omegaSpeed);
      omegaSpeed =MathUtil.applyDeadband(omegaSpeed, 0.008);

      System.out.println("x,y,omega = "+xSpeed+", " +ySpeed+", "+omegaSpeed+" with distance = "+distanceCamToAprilTag+", angle error = "+(goalPose.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees()));
      // once reach goal, should not applied more power

      if(   Math.abs(xSpeed) < 0.015 && Math.abs(ySpeed) < 0.015 && Math.abs(omegaSpeed) < 0.01 ) {
         isGoalReached = true;
      }

      if( isGoalReached == true) {
        // if goal reached before, don't apply new power
        xSpeed = 0;
        ySpeed = 0;
        omegaSpeed = 0;
         System.out.println("Goal is reached with distance = "+distanceCamToAprilTag+", angle error = "+(goalPose.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees()) );
      }

      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()), true, CHASE_TAG_MAX_SPEED
      );
    }
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished(){
    if( (tagTimer + tagTimeout) < Timer.getFPGATimestamp()) {
      // after 20 second, stop command
      
      return true;
    }
    else {
        return isGoalReached;
    }
  }

}
