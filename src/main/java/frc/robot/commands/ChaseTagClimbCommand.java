package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.SwerveSubsystem;


/*
 *  Pretty much the same code as ChaseTagCommand4.java, but using Front Camera instead of Back Camera
 */


public class ChaseTagClimbCommand extends Command {
   
  // front AprilTag Camera -- roll or pitch , which is 30 degree?
  
  public static final double CHASE_TAG_MAX_SPEED = 1.5; // swerve can have max speed 4 or higher, but is not good for chasing tag
  public static final double CHASE_TAG_MAX_PID_OUTPUT = 0.4;

  public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
    new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(20)),
    new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))); 



  private final PhotonCamera photonCamera;
  private final SwerveSubsystem drivetrainSubsystem;
  private Supplier<Pose2d> poseProvider;
 
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

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
  

  public static Map<Integer, Integer> aprilTagIDtoRobotAngle;
  static {
              aprilTagIDtoRobotAngle = new HashMap<>();
              aprilTagIDtoRobotAngle.put(11, 120);
              aprilTagIDtoRobotAngle.put(12, -120);
              aprilTagIDtoRobotAngle.put(13, 0);
              aprilTagIDtoRobotAngle.put(14, 180);
              aprilTagIDtoRobotAngle.put(15, -60);
              aprilTagIDtoRobotAngle.put(16, 60);
  }        
        


  public boolean isGoalReached  = false;


  private Pose3d robotPose3dByVision = null;
  private Pose2d goalPose;
  private double distanceRobotToAprilTag;
  Pose3d aprilTagPose3d = null;
  double  lastGyroYaw = 0.0;
  double  lastDriveStraightAngle = 0.0;  
  double vinniesError = 0.0;
  double angleError = 0.0;
  int lastFiducialId = -1;
  private Transform3d which_tag_to_goal;

  private double tagTimer ;
  private double tagTimeout = 20;

  private boolean driveStraightFlag = false;
  private double driveStraightAngle = 0;
  private boolean useOpenLoop = true;
  private boolean isFieldRelative;

  public ChaseTagClimbCommand(
        PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) 
  {    
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {

    isGoalReached = false;
    tagTimer = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    
    Pose3d aprilTagPose3d = null;
    int fiducialId = -1;
    double joystickX = 0.0;
    double rotationVal = -RobotContainer.oi.driverStick.getRawAxis(RobotContainer.rotationAxis);
    double strafeVal = -RobotContainer.oi.driverStick.getRawAxis(RobotContainer.strafeAxis);
    double translationVal = -RobotContainer.oi.driverStick.getRawAxis(RobotContainer.translationAxis);
    translationVal =  translationLimiter.calculate(  MathUtil.applyDeadband(translationVal, Constants.stickDeadband) );
    strafeVal = strafeLimiter.calculate(  MathUtil.applyDeadband(strafeVal, Constants.stickDeadband));
    rotationVal = rotationLimiter.calculate(   MathUtil.applyDeadband(rotationVal, Constants.stickDeadband));

    boolean hasTarget = false;

    if( photonCamera != null) {
       var results = photonCamera.getLatestResult();
    //if( RobotContainer.photonBackOVCamera != null) {  // temp code
    //  var results = RobotContainer.photonBackOVCamera.getLatestResult();
      if( results.hasTargets() ) {
            var result = results.getBestTarget();
            if( result != null) {
                    hasTarget = true;
                    driveStraightAngle = drivetrainSubsystem.getYawInDegree();
                    lastGyroYaw = driveStraightAngle;
                    // add the vision data
                    driveStraightAngle = driveStraightAngle - result.getYaw()  +6 ;// add or minus need test out
                    driveStraightFlag = true;
                    lastDriveStraightAngle =  driveStraightAngle;

                    fiducialId = result.getFiducialId();
                    lastFiducialId  = fiducialId;
                    
                    aprilTagPose3d = RobotContainer.poseEstimator.aprilTagFieldLayout.getTagPose(fiducialId).get();

                    // optional,  get distance to April Tag by vision,  we also can get the more accurate distance from IR Sensor
                    if (result.getPoseAmbiguity() <= .7 && fiducialId >= 0   && aprilTagPose3d != null   ) {
                      Transform3d camToTarget = result.getBestCameraToTarget();
                      Pose3d camPose3d = aprilTagPose3d.transformBy(camToTarget.inverse());
                      robotPose3dByVision = camPose3d.transformBy(ROBOT_TO_CAMERA_BACK.inverse());//(CAMERA_TO_ROBOT);

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

                      goalPose = aprilTagPose3d.transformBy(which_tag_to_goal).toPose2d();
                      if( robotPose3dByVision != null) {
                        // check distance and 2d angle make sense -- test code
                        distanceRobotToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose3dByVision.toPose2d(), goalPose);
                        //System.out.println("Distance btw robot and goal = "+distanceRobotToAprilTag);    
                      } 
                    }
            }
      }


      

      // if the target is outside the vision, use the last value if driveStraight is still in progress
      if(  driveStraightFlag == true) {
                
                if( hasTarget == true) {
                   vinniesError= driveStraightAngle - drivetrainSubsystem.getYawInDegree()   ;
                  joystickX = vinniesError * 0.025;//0.025;//0.01
                  if(Math.abs(joystickX) > 0.4) {
                      joystickX = Math.signum(joystickX) * 0.4;
                  }
                } 
                else {
                  
                  joystickX = vinniesError * 0.025;//0.025;//0.01
                  if(Math.abs(joystickX) > 0.4) {
                      joystickX = Math.signum(joystickX) * 0.4;
                  }
                  
                  // re-calculate the distance by pose
                  distanceRobotToAprilTag = PoseEstimatorSubsystem.calculateDifference(robotPose2d, goalPose);
                }

                if( hasTarget == true) {
                  rotationVal = joystickX;
                }
                else {
                  rotationVal = 0;
                }
                
                strafeVal = 0;
                isFieldRelative = false;
                

                // in drive straight mode, ignore rotation and strafe from joystick, 
                
                double desiredGyroAngle = aprilTagIDtoRobotAngle.get(lastFiducialId); 
                // slow down at the end
                if( distanceRobotToAprilTag < 1.1) {
                  // IMPORTANT:  need turn off drive straight model near the end !!!
                  driveStraightFlag = false;

                  //translationVal = 0;
                  strafeVal = 0;
                  rotationVal = 0;
                  joystickX = 0;


                  //angleError = (goalPose.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees()) ;
                 
                  
                  // rotate the robot to the desired gyro degree
                  /* 
                  vinniesError = desiredGyroAngle - Math.IEEEremainder( drivetrainSubsystem.getYawInDegree(), 360);
                  if( Math.abs(vinniesError) > 2) {
                    joystickX = vinniesError * 0.015;//0.025;//0.01
                    if(Math.abs(joystickX) > 0.4) {
                        joystickX = Math.signum(joystickX) * 0.4;
                    }
                  }
                  */

                }
                
                
                rotationVal = joystickX;
                strafeVal = 0;
                //strafeVal = xSpeed;

              
                isFieldRelative = false;
                System.out.println("Vision IP Climb driveStraightAngle = "+driveStraightAngle+", desiredGyroAngle = "+desiredGyroAngle+", pid output ="+joystickX+", vision dist = "+distanceRobotToAprilTag+", pigeon Yaw = "+drivetrainSubsystem.getYawInDegree());
            
            
        }
    }


    
    //double angleError = (goalPose.getRotation().getDegrees() - robotPose2d.getRotation().getDegrees());
    

    //System.out.println("Goal Pose = "+ goalPose.toString());
    //System.out.println("robot Pose = "+robotPose2d.toString());
    System.out.println("translationVal,strafeVal,rotationVal = "+translationVal+", " +strafeVal+", "+rotationVal+" with distance = "+distanceRobotToAprilTag+", angle error = "+vinniesError);
    System.out.println("vision distance = "+distanceRobotToAprilTag+", pigeon Yaw = "+drivetrainSubsystem.getYawInDegree());

    if( distanceRobotToAprilTag < 0.8 &&  vinniesError  < 2   ) {
      isGoalReached = true;
    }

    if( isGoalReached == true) {
      // if goal reached before, don't apply new power
      translationVal = 0;
      strafeVal = 0;
      rotationVal = 0;
      PoseEstimatorSubsystem.setLEDColor(Color.kGold);
      System.out.println("Goal is reached with distance = "+distanceRobotToAprilTag+", angle error = "+vinniesError );
    }

    /* Drive */
  
    drivetrainSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                isFieldRelative, 
                useOpenLoop
                
    );
    
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

    // may enhance to check the distance and angle to set isGoalReached = true
    //return false;
  }

}
