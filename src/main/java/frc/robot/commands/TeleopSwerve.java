package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    //private Boolean fieldSentric;
    BooleanSupplier robotCentricSup;
    private BooleanSupplier slowSpeedSup;


    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean isFieldRelative;

    private boolean driveStraightFlag = false;
    private double driveStraightAngle = 0;


    //public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean fieldSentric, BooleanSupplier slowSpeedSup) {
    
    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowSpeedSup) {
        this.s_Swerve = s_Swerve;
        this.slowSpeedSup = slowSpeedSup;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
         //this.fieldSentric = fieldSentric;
    }

    @Override
    public void execute() {
        double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.95 : 0.6; //0.2, 0.5 //0.05, 0.2

        boolean useOpenLoop = true;// false;  // test out the closed loop

        /* Get Values, Deadband*/
        double translationVal = translationLimiter.calculate(speedMultiplier *  MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) );
        double strafeVal = strafeLimiter.calculate( speedMultiplier * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
        double rotationVal = rotationLimiter.calculate(  speedMultiplier * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));

        isFieldRelative = !robotCentricSup.getAsBoolean();

        double joystickX = 0.0;

        // handle drive straight 
        if (RobotContainer.oi.driverStick.getRightTriggerAxis() > 0.4) { //drive straight button
                if (!driveStraightFlag) {
                    driveStraightAngle = s_Swerve.getYawInDegree();
                    driveStraightFlag = true;
                }
                double vinniesError = driveStraightAngle - s_Swerve.getYawInDegree();
                joystickX = vinniesError * RobotMap.driveStraightProportion;

                // in drive straight mode, ignore rotation and strafe
                rotationVal = joystickX;
                strafeVal = 0;
                isFieldRelative = false;
                System.out.println("driveStraightAngle = "+driveStraightAngle+", vinniesError = "+vinniesError+", p ="+joystickX);
        }
        else if (RobotContainer.oi.turnLeft180Button.getAsBoolean() == true) { 

            if( RobotContainer.photonLifeCam != null) {
                var results = RobotContainer.photonLifeCam.getLatestResult();
            //if( RobotContainer.photonBackOVCamera != null) {  // temp code
            //   var results = RobotContainer.photonBackOVCamera.getLatestResult();
               if( results.hasTargets() ) {
                    var result = results.getBestTarget();
                    if( result != null) {
                            driveStraightAngle = s_Swerve.getYawInDegree();
                            // add the vision data
                            driveStraightAngle = driveStraightAngle - result.getYaw();// add or minus need test out
                            driveStraightFlag = true;
                    }
               }

               // if the target is outside the vision, use the last value if driveStraight is still in progress
               if(  driveStraightFlag == true) {
                        double vinniesError = driveStraightAngle - s_Swerve.getYawInDegree();
                        joystickX = vinniesError * 0.01;//0.025;//0.01
                        if(Math.abs(joystickX) > 0.4) {
                            joystickX = Math.signum(joystickX) * 0.4;
                        }

                        // in drive straight mode, ignore rotation and strafe
                        rotationVal = joystickX;
                        strafeVal = 0;
                        if( translationVal > 0.4) {
                            translationVal = 0.4; // fix the speed too?
                        }
                        isFieldRelative = false;
                        System.out.println("Vision IP driveStraightAngle = "+driveStraightAngle+", vinniesError = "+vinniesError+", pid output ="+joystickX);
                }
            }
        }
        else {
        driveStraightFlag = false;
        }



        /* Drive */
  
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                isFieldRelative, 
                useOpenLoop
                
            );

       
    }
}