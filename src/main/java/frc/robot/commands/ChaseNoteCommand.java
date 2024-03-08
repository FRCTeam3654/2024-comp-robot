package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
//import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ChaseNoteCommand extends Command {
   
  
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-6), Units.inchesToMeters(7), Units.inchesToMeters(22)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))); 


  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(22);// same as the camera height in above ROBOT_TO_CAMERA
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-12); // camera facing down, about 12 degree from vertical line

  final double GOAL_ROBOT_TO_NOTE_DISTANCE_METERS = Units.inchesToMeters(10);
  //final double MINIMIUM_CAMERA_TO_NOTE_DISTANCE = Units.inchesToMeters(20); //17; varies by each robot/camera setup




  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);//(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);//(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(2, 2);//(8, 8);



  private final PhotonCamera photonLifeCam;
  private final SwerveSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider; // tell me where my robot is

  private final ProfiledPIDController xController = new ProfiledPIDController(0.75, 0, 0, X_CONSTRAINTS); //(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.75, 0, 0, Y_CONSTRAINTS); //(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.35, 0, 0, OMEGA_CONSTRAINTS); // (2, 0, 0, OMEGA_CONSTRAINTS);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private PhotonTrackedTarget lastNote;
  private Pose2d lastNotPose2d;
  private Pose2d lastGoalPose2d;
  private double distanceToGo = 0.0;

  public boolean isGoalReached  = false;

  
  
  public ChaseNoteCommand(
        PhotonCamera photonLifeCam, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
          
    this.photonLifeCam = photonLifeCam;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);//0.2
    yController.setTolerance(0.2);//0.2
    omegaController.setTolerance(Units.degreesToRadians(3));//3
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    lastNote = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    isGoalReached = false;
  }

  @Override
  public void execute() {
   
    double angleDiff = 360;// max the angle difference as default
    var robotPose2d = poseProvider.get();
    System.out.println("robotPose2d = "+robotPose2d.toString());

    if( lastGoalPose2d != null) {
      System.out.println("lastGoalPose2d = "+lastGoalPose2d.toString());
    }

    // robot is in 2 dimensional (X,Y), convert to Pose3d for reusing some math
    var robotPose3d = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    if( photonLifeCam != null) {
        var results = photonLifeCam.getLatestResult();
        if (results.hasTargets()) {
          // Find the notes we want to chase

          // photonRes.getBestTarget().getYaw() actually works (left side is negative in our system)
        //  System.out.println("LifeCam found Yaw = "+results.getBestTarget().getYaw()+", size = "+results.getTargets().size());
          SmartDashboard.putNumber("YawFromLifeCam", results.getBestTarget().getYaw());

          // Transform the robot's pose to find the camera's pose
          var cameraPose3d = robotPose3d.transformBy(ROBOT_TO_CAMERA);
          var cameraPose2d = cameraPose3d.toPose2d();
          

          var result = results.getBestTarget();

          //MUST HANDLE THE FROZEM IMAGE //////////
          if (result != null && result.equals(lastNote)) {
            //     System.out.println("Frozen image ...");
            if( lastNotPose2d != null) {
              // use current robot pose to calculate the goal
                var distanceNow = Math.sqrt(   (lastNotPose2d.getX() - robotPose2d.getX() ) * (lastNotPose2d.getX() - robotPose2d.getX() ) + (lastNotPose2d.getY() - robotPose2d.getY() ) * (lastNotPose2d.getY() - robotPose2d.getY() )  );
                double range = distanceNow;
                if( distanceNow > GOAL_ROBOT_TO_NOTE_DISTANCE_METERS) {
                    range = distanceNow - GOAL_ROBOT_TO_NOTE_DISTANCE_METERS; // stop right in front of note, rather than drive over it
                    distanceToGo = range;
                }
                else {
                    
                    range = 0;
                    

                    distanceToGo = range;

                    Rotation2d noteRotation2d = lastNotPose2d.getRotation();
              
                    // find the goal's pose (a small distance from note center)
                    double noteGoalX = robotPose2d.getX()  +  range * Math.cos(noteRotation2d.getRadians());
                    double noteGoalY = robotPose2d.getY()  +  range * Math.sin(noteRotation2d.getRadians());
                    Pose2d goalPose2d = new Pose2d(noteGoalX, noteGoalY, noteRotation2d);
                    lastGoalPose2d  = goalPose2d;
              }
            }
              
              
          }        
          else if (result != null && !result.equals(lastNote)) {
        
            // This is new target data, so recalculate the goal
            lastNote = result;

            // need verfiy with camera: which direction is postive:  for postive robot angle - getYaw,  for negetive + getYaw  in continuous mode ( )
            double yaw = (-1) * result.getYaw(); //   BE CAREFUL and VERIFY if need (-1)


            //double yawInRadian = Units.degreesToRadians(yaw) ;
            double rangeNote =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getPitch()));

            double range = rangeNote;
            if( rangeNote > GOAL_ROBOT_TO_NOTE_DISTANCE_METERS) {
                range = rangeNote - GOAL_ROBOT_TO_NOTE_DISTANCE_METERS; // stop right in front of note, rather than drive over it
            }
            else {
              range = 0;// avoid sudden moving back
              distanceToGo = 0;
              isGoalReached = true;
            }
           

            // range is not very accurate due to the inaccuracy of measured pitch, but good enough to start
            SmartDashboard.putString("Distance--Angle = ", rangeNote + "--"+ yaw);
            System.out.println("Distance--Angle = "+rangeNote + "--"+ yaw);

            
            double cameraPoseAngle =  cameraPose2d.getRotation().getDegrees();//.getRadians(); 
            double clampedNoteAngle = cameraPoseAngle;
            if( cameraPoseAngle > 0 ) {
                  clampedNoteAngle = Math.IEEEremainder(cameraPoseAngle+yaw, 360); // in degree Math.IEEReminder(angle, 360)
            }
            else if( cameraPoseAngle < 0 ) {
                  clampedNoteAngle = Math.IEEEremainder(cameraPoseAngle+yaw, 360); // in degree
            }
            else {
              clampedNoteAngle = yaw;
            }
            
            double clampedNoteAngleRadian  = Units.degreesToRadians(clampedNoteAngle); 
            Rotation2d noteRotation2d = new Rotation2d(clampedNoteAngleRadian);

            // find the goal's pose (a small distance from note center), should use cameraPose2d  instad robotPose2d
            double noteGoalX = cameraPose2d.getX()  +  range * Math.cos(clampedNoteAngleRadian);
            double noteGoalY = cameraPose2d.getY()  +  range * Math.sin(clampedNoteAngleRadian);
            
            //double noteGoalX = robotPose2d.getX()  +  range * Math.cos(clampedNoteAngleRadian);
            //double noteGoalY = robotPose2d.getY()  +  range * Math.sin(clampedNoteAngleRadian);
             


            Pose2d goalPose2d = new Pose2d(noteGoalX, noteGoalY, noteRotation2d);
            lastGoalPose2d  = goalPose2d;
            // temp
            lastNotPose2d = goalPose2d;

            System.out.println("goalPose2d = " + goalPose2d.toString());
            SmartDashboard.putString("goalPose2d", goalPose2d.toString());

            /* 
            // find the note's pos, use note's center , the full distance found by camera
            double noteX = cameraPose2d.getX()  +  rangeNote * Math.cos(clampedNoteAngleRadian);
            double noteY = cameraPose2d.getY()  +  rangeNote * Math.sin(clampedNoteAngleRadian);
          

            Pose2d notePose2d = new Pose2d(noteX, noteY, noteRotation2d);
            lastNotPose2d = notePose2d;
            System.out.println("note pose2d = " + notePose2d.toString());
           */
            
            // for omegaController, the goal is not set the angle to that of note, 
            // the goal is:  robot to note's angle  ==  robot's pose angle
            //double robotToNoteAngle = Math.atan( (robotPose2d.getY() - lastNotPose2d.getY() ) /  ( robotPose2d.getX() - lastNotPose2d.getX()   )   );
            
            // angleDiff for info/debug only
           //angleDiff =  robotToNoteAngle - robotPose2d.getRotation().getRadians()  ; // which one first, this dertermines the sign
     //       System.out.println("angleDiff = " + Units.radiansToDegrees(angleDiff) );
           // SmartDashboard.putNumber("angleDiff", Units.radiansToDegrees(angleDiff));
            



            // Drive
            xController.setGoal(noteGoalX);
            yController.setGoal(noteGoalY);
            //omegaController.setGoal(noteRotation2d.getRadians());
            omegaController.setGoal(clampedNoteAngleRadian);
          }
        }
        else {
          System.out.println("No object found by LifeCam");
        }
    }
    else {
      System.out.println("LifeCam Obj is null");
    }
    
    
    if (lastNotPose2d == null) {
      // No target has been visible, check current pose2d, verfiy the distance < threhold
      System.out.println("no last note pose");
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose2d.getX());
      if (xController.atGoal() || distanceToGo < 0.1 ) {
      //if (xController.atGoal() || distanceToGo < 0.1 ) {
      //if ( distanceToGo < 0.15 ) {
        xSpeed = 0;
        System.out.println("xController at Goal");
      }

      var ySpeed = yController.calculate(robotPose2d.getY());
      if (yController.atGoal() || distanceToGo < 0.1) {
      //if ( distanceToGo < 0.15) {
        ySpeed = 0;
        System.out.println("yController at Goal");
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      //if ( Math.abs(angleDiff) < 3) {
      if (omegaController.atGoal() ) {
        omegaSpeed = 0;
        System.out.println("omegaController at Goal");
      }


      xSpeed = xLimiter.calculate( MathUtil.applyDeadband(xSpeed, 0.01) );
      ySpeed = yLimiter.calculate(  MathUtil.applyDeadband(ySpeed, 0.01));
      omegaSpeed =MathUtil.applyDeadband(omegaSpeed, 0.018);

      System.out.println("x,y,omega = "+xSpeed+", " +ySpeed+", "+omegaSpeed);

      if( xController.atGoal() &&  Math.abs(xSpeed) < 0.001 && Math.abs(ySpeed) < 0.001 && Math.abs(omegaSpeed) < 0.001 ) {
        isGoalReached = true;
      }

      if( isGoalReached == true) {
        // if goal reached before, don't apply new power
        xSpeed = 0;
        ySpeed = 0;
        omegaSpeed = 0;
      }

      //lastGoalPose2d  vs robotPose2d.getRotation ?
      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,  robotPose2d.getRotation()), true
      );
    }
    

  }


  private static double clampTo180(double angle) {
    double value = angle;

    value = angle % 360;
    if (value > 180) {
        value = -360 + value;
    } 
    else if(value < -180) {
        value = 360 + value;
    }
    return value;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }


  @Override
  public boolean isFinished(){
        return isGoalReached;
  }

  public static void main(String[] args) {
    
    
   
  }

}
