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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand extends Command {
   
  // front AprilTag Camera -- roll or pitch , which is 30 degree?
  // // pitch is the Y angle, and it is positive down https://github.com/ligerbots/Crescendo2024/blob/main/src/main/java/frc/robot/subsystems/AprilTagVision.java

  public static final Transform3d ROBOT_TO_CAMERA_FRONT = new Transform3d(
        new Translation3d(Units.inchesToMeters(-7), Units.inchesToMeters(3.5), Units.inchesToMeters(21)),
        new Rotation3d(0, Units.degreesToRadians(-30), 0)); 

  // back camera's pitch: is it -30 degree since it is looking up , ( not 150 since yaw turns 180 degree, pitch stays 30 degree looking up) : decouple pitch from yaw
  public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(20)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))); 

  // should not change much, NEED match the max speed
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);//(1.5,2),(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);//(1.5, 2),(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(3, 4);//(2,2),(8, 8);
  

  // speaker's TAG_TO_GOAL
  //private static final Transform3d TAG_TO_GOAL = 
  //    new Transform3d(
  //        new Translation3d(1.3, 0.0, 0.0),
  //        new Rotation3d(0.0, 0.0, Math.PI));

  // drive backward,use back camera to align to speaker
  private static final Transform3d TAG_TO_GOAL_SPEAKER = 
      new Transform3d(
          new Translation3d(1.3, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0));

  // AMP's TAG_TO_GOAL
  private static final Transform3d TAG_TO_GOAL_AMP = 
      new Transform3d(
          new Translation3d(Units.inchesToMeters(26), 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0));
  

  private Transform3d which_tag_to_goal;

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is



  private final ProfiledPIDController xController = new ProfiledPIDController(0.75, 0, 0, X_CONSTRAINTS); //2,(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.75, 0, 0, Y_CONSTRAINTS); //2,(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.35, 0, 0, OMEGA_CONSTRAINTS); //0.3;// 0.5,(2, 0, 0, OMEGA_CONSTRAINTS);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  public boolean isGoalReached  = false;

  private PhotonTrackedTarget lastTarget;
  private Pose3d robotPose3dByVision = null;
  

  private double tagTimer ;
  private double tagTimeout = 8;
  
  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);//0.2
    yController.setTolerance(0.2);//0.2
    omegaController.setTolerance(Units.degreesToRadians(5));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    isGoalReached = false;

    tagTimer = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    
    var robotPose2d = poseProvider.get();
    //System.out.println("robotPose2d = "+robotPose2d.toString());
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
          //.filter(t -> t.getFiducialId() == TAG_TO_CHASE)
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
        
        // need change old way: chaseTag (tag position is unknow, need based on robot) ==> here Tag position is known, just directly ask robot to move to it
        // what if the poseEstimate is not accurate, the calculated tag position not match the AprilTag

        // duplicate the poseEstimator's logic to get the Robot's pose3d
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




        // Transform the robot's pose to find the camera's pose
        //var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        //var camToTarget = target.getBestCameraToTarget();
        //var targetPose = cameraPose.transformBy(camToTarget);

        //System.out.println("Target = "+targetPose.toString());
        /////////////////////////////////////////////////////////////////////////////////////
        // logic is enhanced here: not just only see the target tag,  enhanced to : if see other tag not target tag, use that data to move to the target data 
        //if(target.getFiducialId() != TAG_TO_CHASE) {
            // targetPose is Tag 1, not TAG_TO_CHASE 4, so need transform from tag1 to tag4 pose
           // targetPose = targetPose.transformBy(PoseEstimatorSubsystem.tag1Totag4);
        //}
        /////////////////////////////////////////////////////////////////////////////////////



        if(fiducialId == 4 || fiducialId == 7 ) {
            // speaker
             which_tag_to_goal = TAG_TO_GOAL_SPEAKER;
        }
        else if(fiducialId == 5 || fiducialId == 6 ) {
            // AMP
             which_tag_to_goal = TAG_TO_GOAL_AMP;
        }

        
        // Transform the tag's pose to set our goal
        var goalPose = aprilTagPose3d.transformBy(which_tag_to_goal).toPose2d();

        System.out.println("Goal = "+ goalPose.toString());

        System.out.println("robotPose2d = "+robotPose2d.toString());

        if( robotPose3dByVision != null) {
            //System.out.println("robotPose3dByVision = "+robotPose3dByVision.toString());

            // check distance and 2d angle make sense -- test code
            double distanceCamToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose3dByVision.toPose2d(), goalPose);
            System.out.println("Distance btw robot and goal = "+distanceCamToAprilTag);
            if(distanceCamToAprilTag > 0.5) {
              isGoalReached = false;
            }
        }

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
      //drivetrainSubsystem.stop();
    } else {
      // Drive to the target
     
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
        System.out.println("xController at Goal");
      }

      var ySpeed = yController.calculate(robotPose.getY());
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
      //xSpeed = translationLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.02) );
      //ySpeed = strafeLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.02));
      //omegaSpeed = rotationLimiter.calculate(  MathUtil.applyDeadband(omegaSpeed, 0.01));

      xSpeed = xLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.01) );
      ySpeed = yLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.01));
      //omegaSpeed = omegaLimiter.calculate(  omegaSpeed);
      omegaSpeed =MathUtil.applyDeadband(omegaSpeed, 0.018);

      System.out.println("x,y,omega = "+xSpeed+", " +ySpeed+", "+omegaSpeed);
      // once reach goal, should not applied more power
      if( xController.atGoal() &&  Math.abs(xSpeed) < 0.001 && Math.abs(ySpeed) < 0.001 && Math.abs(omegaSpeed) < 0.001 ) {
        isGoalReached = true;
      }
      else {
        
      }

      if( isGoalReached == true) {
        // if goal reached before, don't apply new power
        xSpeed = 0;
        ySpeed = 0;
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()), true
      );
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    //isGoalReached = false;
    //drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished(){
    if( (tagTimer + tagTimeout) < Timer.getFPGATimestamp()) {
      // after 8 second, stop command
      
      return true;
    }
    else {
        return isGoalReached;
    }
  }

  public static void main(String[] args) {
  
  }

}
