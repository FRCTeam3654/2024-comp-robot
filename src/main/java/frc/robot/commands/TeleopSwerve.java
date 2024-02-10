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
    private Boolean fieldSentric;
    private BooleanSupplier slowSpeedSup;


    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean isFieldRelative;

    private boolean driveStraightFlag = false;
    private double driveStraightAngle = 0;


    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean fieldSentric, BooleanSupplier slowSpeedSup) {
        this.s_Swerve = s_Swerve;
        this.slowSpeedSup = slowSpeedSup;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldSentric = fieldSentric;

        isFieldRelative = fieldSentric;
    }

    @Override
    public void execute() {
        double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.8 : 0.5; //0.2, 0.5 //0.05, 0.2

        /* Get Values, Deadband*/
        double translationVal = translationLimiter.calculate(speedMultiplier *  MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) );
        double strafeVal = strafeLimiter.calculate( speedMultiplier * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
        double rotationVal = rotationLimiter.calculate(  speedMultiplier * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));


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
        else {
        driveStraightFlag = false;
        }



        /* Drive */
        //if (!slowSpeedSup.getAsBoolean()){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                isFieldRelative, 
                true
                
            );

        /* Slow mode drive */
        /* 
        }else{
            s_Swerve.drive(
                //new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * Constants.Swerve.XYSlowRatio),
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * speedMultiplier), 
                //rotationVal * Constants.Swerve.maxAngularVelocity * Constants.Swerve.rotationSlowRatio, 
                rotationVal * Constants.Swerve.maxAngularVelocity * speedMultiplier, 
                fieldSentric, 
                true,
                true
            );
        }
        */
    }
}