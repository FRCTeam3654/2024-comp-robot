package frc.robot.subsystems;

import  static frc.robot.commands.ChaseTagCommand.ROBOT_TO_CAMERA_FRONT;
import  static frc.robot.commands.ChaseTagCommand.ROBOT_TO_CAMERA_BACK;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

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
import edu.wpi.first.wpilibj.Timer;
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
  //public static final HashMap<Integer, Pose3d> targetPoses =  new HashMap<Integer, Pose3d>() {{
    //put(2, new Pose3d(Units.inchesToMeters(60),0,Units.inchesToMeters(10.5), new Rotation3d(0,0,Units.degreesToRadians(180))));
    //put(1, new Pose3d(Units.inchesToMeters(-101),Units.inchesToMeters(49),Units.inchesToMeters(8.5), new Rotation3d(0,0,Units.degreesToRadians(0))));
  //}}   ;   
  
  //public static Transform3d tag4Totag2 = targetPoses.get(2).minus(targetPoses.get(4));
  //public static Transform3d tag2Totag4 = tag4Totag2.inverse();

  public final AprilTagFieldLayout aprilTagFieldLayout;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  public static final Vector<N3> stateStdDevs = frc.robot.Constants.Vision.stateStdDevs;//VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  public static final Vector<N3> visionMeasurementStdDevs = frc.robot.Constants.Vision.visionMeasurementStdDevs;//VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  public final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  private int noCameraCycleCnt = 0;


  private  PhotonPoseEstimator m_photonPoseEstimatorFront = null ;
  private  PhotonPoseEstimator m_photonPoseEstimatorBack = null;

  // when newColor is null, use the default logic to set LED color
  // When newColor is not null, that is the color that LED is set with 3 seconds timeout
  public static  Color  newColor = null ; // can be updated by other system or command
  private static double colorTimer = 0.0 ; // start with big number
  private static double colorTimeout = 3;//seconds



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
   

     //m_photonPoseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout,
     //               PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonFrontOVCamera, ROBOT_TO_CAMERA_FRONT);

     m_photonPoseEstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonBackOVCamera, ROBOT_TO_CAMERA_BACK);


     //photonFrontOVCamera.setDriverMode(false);
     photonBackOVCamera.setDriverMode(false);

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
    boolean useFrontCamera = true;
    Transform3d whichROBOT_TO_CAMERA = ROBOT_TO_CAMERA_BACK;
    //int whichCameraToUse = 0; // 0 -- None ;  1 -- Front Camera ;  2 -- Back Camera

    //if(photonFrontOVCamera != null ) {
    if(photonFrontOVCamera != null && photonFrontOVCamera.isConnected() ) {
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
                useFrontCamera = true;
                whichROBOT_TO_CAMERA = ROBOT_TO_CAMERA_FRONT;
            }
          }
          
      }
    }

    if(photonBackOVCamera != null ) {
        pipelineResult = photonBackOVCamera.getLatestResult();
        resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) { 
            target = pipelineResult.getBestTarget();
            if( target.getFiducialId() >= 0 ) {
              fiducialId = target.getFiducialId();
              backCameraPoseAmbiguity = target.getPoseAmbiguity();
              if( !isAboutEqual(backCameraPoseAmbiguity, -1.0) ) {
                  if ( isAboutEqual(frontCameraPoseAmbiguity, -1.0) ) {
                    photonCamera = photonBackOVCamera;
                    useFrontCamera = false;
                    whichROBOT_TO_CAMERA = ROBOT_TO_CAMERA_BACK;
                  }
                  else if (backCameraPoseAmbiguity < frontCameraPoseAmbiguity)   {
                     photonCamera = photonBackOVCamera;
                     useFrontCamera = false;
                     whichROBOT_TO_CAMERA = ROBOT_TO_CAMERA_BACK;
                  } 
              }
            }
        }
    }

    // check the color
    if( (colorTimer + colorTimeout) < Timer.getFPGATimestamp()) {
        newColor = null;
        if( photonCamera == null) {
          // no AprilTag
          RobotContainer.led.setAll(Color.kFuchsia);
        }
        colorTimer =  Timer.getFPGATimestamp();
    }

    if( newColor != null) {     
            if( newColor.toString().equalsIgnoreCase("kLightSkyBlue") ) {
                RobotContainer.led.setRainBowColor();
            }
            else {
              RobotContainer.led.setAll(newColor);
            }
    }

    if(photonCamera == null ) {
        poseEstimator.update(drivetrainSubsystem.getYaw(), drivetrainSubsystem.getModulePositions());

        noCameraCycleCnt++;
        if(noCameraCycleCnt > 3) {
          //RobotContainer.led.setAll(Color.kBlack);
          RobotContainer.led.setAll(Color.kFuchsia);
          //RobotContainer.ledLeft.setAll(Color.kLemonChiffon);
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

        if( newColor == null) {  
            // Color:  Speaker -> Green; Amp --> Blue;  Note --> Red;   Source --> Yellow; Stage --> 
            if( fiducialId == 4 || fiducialId == 7 ) {
              // speaker, possible other two:  3, 8
              RobotContainer.led.setAll(Color.kGreen); 
            }
            else if( fiducialId == 5  || fiducialId == 6) {
              // AMP
              RobotContainer.led.setAll(Color.kBlue);
            }
            else if( fiducialId == 1  || fiducialId == 9) {
              // source, possible other two: 2, 10
              RobotContainer.led.setAll(Color.kYellow);
            }
            else if( fiducialId >= 11) {
              // stage
              RobotContainer.led.setAll(Color.kYellow);
            }
            else{
              RobotContainer.led.setAll(Color.kFuchsia);
            }
        }


       
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
         
        targetPose = this.aprilTagFieldLayout.getTagPose(fiducialId).get();
       

        //if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
          if (target.getPoseAmbiguity() <= .7 && fiducialId >= 0   && targetPose != null   ) {
          Transform3d camToTarget = target.getBestCameraToTarget();

          // here:  front camera is different from back camera:  ROBOT_TO_CAMERA constant is different
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
         
          // check distance and 2d angle make sense -- test code
          double distanceCamToAprilTag = calculateDifference(camPose, targetPose);
          //System.out.println("in poseEstimator, distance btw cam and AprilTag = " + distanceCamToAprilTag);


          var visionMeasurement = camPose.transformBy(whichROBOT_TO_CAMERA.inverse());//(CAMERA_TO_ROBOT);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);

          // possible different way -- let PhotonPoseEstimator handle the transform
          //updateOdometry(poseEstimator,  useFrontCamera);

        }
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getYaw(),
      drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }



  //  LED  Color related
  public static  void setLEDColor(Color new_Color){
    colorTimer = Timer.getFPGATimestamp();
    newColor = new_Color;
 }

 public static void setDefaultLEDColor(){
    newColor = null;
 }

 public static void setRainBowLEDColor(){
   // use a special color as RowBow
    colorTimer = Timer.getFPGATimestamp();
    newColor = Color.kLightSkyBlue; 
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


   // get the pose for a tag. https://github.com/ligerbots/Crescendo2024/blob/main/src/main/java/frc/robot/subsystems/AprilTagVision.java
    // will return null if the tag is not in the field map (eg -1)
    public Optional<Pose2d> getTagPose(int tagId) {
      // optional in case no target is found
      Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
      if (tagPose.isEmpty()) {
          return Optional.empty(); // returns an empty optional
      }
      return Optional.of(tagPose.get().toPose2d());
  }

  // Private routines for calculating the odometry info

  public static double calculateDifference(Pose3d x, Pose3d y) {
      return x.getTranslation().getDistance(y.getTranslation());
  }

  public static double calculateDifference(Pose2d x, Pose2d y) {
    return x.getTranslation().getDistance(y.getTranslation());
}

  private Optional<EstimatedRobotPose> getEstimateForCamera(PhotonCamera cam, PhotonPoseEstimator poseEstimator, Pose2d robotPose) {
    if (!cam.isConnected()) return Optional.empty();

    try {
        poseEstimator.setReferencePose(robotPose);
        return poseEstimator.update();
    } catch (Exception e) {
        // bad! log this and keep going
        DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
        return Optional.empty();
    }
  }

  public void updateOdometry(SwerveDrivePoseEstimator odometry, boolean useFrontCamera) {
        // Cannot do anything if there is no field layout
        if (aprilTagFieldLayout == null)
            return;

        

        // Warning: be careful about fetching values. If cameras are not connected, you get errors
        // Example: cannot fetch timestamp without checking for the camera.
        // Make sure to test!

        Pose2d robotPose = odometry.getEstimatedPosition();
        if( useFrontCamera == true) {
            Optional<EstimatedRobotPose> frontEstimate = 
                    getEstimateForCamera(photonFrontOVCamera, m_photonPoseEstimatorFront, robotPose);
           
            if (frontEstimate.isPresent()) {
              Pose2d pose = frontEstimate.get().estimatedPose.toPose2d();
              odometry.addVisionMeasurement(pose, photonFrontOVCamera.getLatestResult().getTimestampSeconds());
            }                            
            return;
        }
        else {
            
            Optional<EstimatedRobotPose> backEstimate = 
                getEstimateForCamera(photonBackOVCamera, m_photonPoseEstimatorBack, robotPose);
    
            if (backEstimate.isPresent()) {
                Pose2d pose = backEstimate.get().estimatedPose.toPose2d();
                odometry.addVisionMeasurement(pose, photonBackOVCamera.getLatestResult().getTimestampSeconds());   
            } 
            return;
        }
    }

    
}
