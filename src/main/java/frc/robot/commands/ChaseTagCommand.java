package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand extends Command {
   
  // front AprilTag Camera
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-7), Units.inchesToMeters(3.5), Units.inchesToMeters(21)),
        new Rotation3d(0, Units.degreesToRadians(30), 0)); 

  public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(20)),
        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180))); 

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);//(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);//(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(2, 2);//(8, 8);
  

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
          new Translation3d(Units.inchesToMeters(16), 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0));
  

  private Transform3d which_tag_to_goal;

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is



  private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS); //(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS); //(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.5, 0, 0, OMEGA_CONSTRAINTS); // (2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;
  
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
          //.filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .7 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        


        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        //System.out.println("Target = "+targetPose.toString());
        /////////////////////////////////////////////////////////////////////////////////////
        // logic is enhanced here: not just only see the target tag,  enhanced to : if see other tag not target tag, use that data to move to the target data 
        //if(target.getFiducialId() != TAG_TO_CHASE) {
            // targetPose is Tag 1, not TAG_TO_CHASE 4, so need transform from tag1 to tag4 pose
           // targetPose = targetPose.transformBy(PoseEstimatorSubsystem.tag1Totag4);
        //}
        /////////////////////////////////////////////////////////////////////////////////////



        if(target.getFiducialId() == 4 || target.getFiducialId() == 7 ) {
            // speaker
             which_tag_to_goal = TAG_TO_GOAL_SPEAKER;
        }
        else if(target.getFiducialId() == 5 || target.getFiducialId() == 6 ) {
            // speaker
             which_tag_to_goal = TAG_TO_GOAL_AMP;
        }

        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(which_tag_to_goal).toPose2d();

       // System.out.println("Goal = "+ goalPose.toString());

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
      

      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation())
      );
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    //drivetrainSubsystem.stop();
  }


  public static void main(String[] args) {
  
  }

}
