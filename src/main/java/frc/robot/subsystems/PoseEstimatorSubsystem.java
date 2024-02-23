package frc.robot.subsystems;

import  static frc.robot.commands.ChaseTagCommand.ROBOT_TO_CAMERA;

import java.io.IOException;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.DrivetrainConstants;
import java.util.List;
import java.util.HashMap;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonFrontOVCamera;
  private final PhotonCamera photonBackOVCamera;
  //private final Swerve drivetrainSubsystem;
  private final SwerveSubsystem drivetrainSubsystem;


  //private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of
  //(
  //  new Pose3d(Units.inchesToMeters(102),0,Units.inchesToMeters(22.5), new Rotation3d(0,0,Units.degreesToRadians(180)))
  //)
  //);

  // temp code to set up the ApriTag postion at shop
  public static final HashMap<Integer, Pose3d> targetPoses =  new HashMap<Integer, Pose3d>() {{
    put(2, new Pose3d(Units.inchesToMeters(60),0,Units.inchesToMeters(10.5), new Rotation3d(0,0,Units.degreesToRadians(180))));
    put(1, new Pose3d(Units.inchesToMeters(-101),Units.inchesToMeters(49),Units.inchesToMeters(8.5), new Rotation3d(0,0,Units.degreesToRadians(0))));
  }}   ;   
  
  //public static Transform3d tag4Totag2 = targetPoses.get(2).minus(targetPoses.get(4));
  //public static Transform3d tag2Totag4 = tag4Totag2.inverse();

  private final AprilTagFieldLayout aprilTagFieldLayout;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  private int noCameraCycleCnt = 0;

  public PoseEstimatorSubsystem(PhotonCamera photonFrontOVCamera, PhotonCamera photonBackOVCamera, SwerveSubsystem drivetrainSubsystem) {
    this.photonFrontOVCamera = photonFrontOVCamera;
    this.photonBackOVCamera = photonBackOVCamera;

    this.drivetrainSubsystem = drivetrainSubsystem;


    

    
    AprilTagFieldLayout layout;
    try {
      //layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      layout =  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      //var alliance = DriverStation.getAlliance();
      //layout.setOrigin(alliance == Alliance.Blue ?
      //    OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;
   

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    // here new Pose2d() means the robot starts at (0,0), which is not true in field

    poseEstimator =  new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        drivetrainSubsystem.getYaw(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target of Front and Back camera
    PhotonPipelineResult pipelineResult;
    double resultTimestamp;
    PhotonCamera photonCamera = null;// photonFrontOVCamera; // pick a default
    PhotonTrackedTarget  target;
    int fiducialId = -1;
    double frontCameraPoseAmbiguity = -1.0;
    double backCameraPoseAmbiguity = -1.0;
    Pose3d targetPose;
    //int whichCameraToUse = 0; // 0 -- None ;  1 -- Front Camera ;  2 -- Back Camera

    if(photonFrontOVCamera != null ) {
      pipelineResult = photonFrontOVCamera.getLatestResult();
      resultTimestamp = pipelineResult.getTimestampSeconds();
      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
          //previousPipelineTimestamp = resultTimestamp;
          target = pipelineResult.getBestTarget();
          if( target.getFiducialId() >= 0 ) {
            fiducialId = target.getFiducialId();
            frontCameraPoseAmbiguity = target.getPoseAmbiguity();
            // maybe need distance as another parameter besides PoseAmbiguity
            if( !isAboutEqual(frontCameraPoseAmbiguity, -1.0) ) {
                photonCamera = photonFrontOVCamera;
            }
          }
          
      }
    }

    if(photonBackOVCamera != null ) {
        pipelineResult = photonFrontOVCamera.getLatestResult();
        resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) { 
            target = pipelineResult.getBestTarget();
            if( target.getFiducialId() >= 0 ) {
              fiducialId = target.getFiducialId();
              backCameraPoseAmbiguity = target.getPoseAmbiguity();
              if( !isAboutEqual(backCameraPoseAmbiguity, -1.0) ) {
                  if ( isAboutEqual(frontCameraPoseAmbiguity, -1.0) ) {
                    photonCamera = photonBackOVCamera;
                  }
                  else if (backCameraPoseAmbiguity < frontCameraPoseAmbiguity)   {
                     photonCamera = photonBackOVCamera;
                  } 
              }
            }
        }
    }


    if(photonCamera == null ) {
        noCameraCycleCnt++;
        if(noCameraCycleCnt > 3) {
          RobotContainer.led.setAll(Color.kBlack);
        }
    } 
    else if(photonCamera != null ) {
      noCameraCycleCnt = 0;
      
      pipelineResult = photonCamera.getLatestResult();
      resultTimestamp = pipelineResult.getTimestampSeconds();
      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        previousPipelineTimestamp = resultTimestamp;
        target = pipelineResult.getBestTarget();
        fiducialId = target.getFiducialId();

        // Color:  Speaker -> Green; Amp --> Blue;  Note --> Red;   Source --> Yellow  
        if( fiducialId == 2 ) {
          RobotContainer.led.setAll(Color.kGreen); 
        }
        else if( fiducialId == 1 ) {
          RobotContainer.led.setAll(Color.kBlue);
        }


        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        
        //Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
        
        targetPose = this.aprilTagFieldLayout.getTagPose(fiducialId).get();
        //targetPose = targetPoses.get(fiducialId); // for real, need use fieldlayout, and fiducialId 
        
      

        //if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
          if (target.getPoseAmbiguity() <= .7 && fiducialId >= 0   && targetPose != null   ) {
          //var targetPose = tagPose.get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());


          var visionMeasurement = camPose.transformBy(ROBOT_TO_CAMERA.inverse());//(CAMERA_TO_ROBOT);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
        }
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getYaw(),
      drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrainSubsystem.getYaw(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }


  public static boolean isAboutEqual(double a, double b) {
    return ( Math.abs(a - b) < 0.000001 );
  }
}
