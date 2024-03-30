package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class TurnToAbsoluteAngleCommand extends Command {

    private final SwerveSubsystem m_robotDrive;
    private boolean complete = false;
    private double angle;
    private double gyro_angle;
    private double turnTimer ;
    private double turnTimeout = 2;

    public TurnToAbsoluteAngleCommand(SwerveSubsystem subsystem, double degrees, double timeoutS){
        m_robotDrive = subsystem;
        angle = degrees;
        turnTimeout = timeoutS;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
               if( alliance.get() == DriverStation.Alliance.Red ) {
                    // red alliance is different from blue alliance
                    angle = (-1) * angle;
               }
        }
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        turnTimer = Timer.getFPGATimestamp();
        complete = false;
        //gyro_angle = angle+m_robotDrive.getYaw().getDegrees(); // need start from current angle + degrees
        gyro_angle = angle;
    
    }
    
    @Override
    public void execute(){
        
        double gyroAngle = Math.IEEEremainder( m_robotDrive.getYaw().getDegrees(), 360);

        final double kP = 0.1; // 0.028;
        SmartDashboard.putNumber("gyroAngle", gyroAngle);
       

        double err = gyro_angle - gyroAngle;
        //double speed = MathUtil.clamp(err * kP, -Constants.Swerve.maxAngularVelocity*0.32, Constants.Swerve.maxAngularVelocity*0.32);
        double speed = MathUtil.clamp(err * kP, -Constants.Swerve.maxAngularVelocity*0.5, Constants.Swerve.maxAngularVelocity*0.5);
    
        if (Math.abs(err) > 2 && ( (turnTimer + turnTimeout) > Timer.getFPGATimestamp() )  )  {
            m_robotDrive.drive(new Translation2d(0,0), speed, false, true);
        } else {
            complete = true;
            System.out.println("turn completed with err = "+err);
        } 
    } 

    @Override
    public void end(boolean inturrupted){
        m_robotDrive.drive(new Translation2d(0,0), 0, false, true);
    }

    @Override
    public boolean isFinished(){
        if( (turnTimer + turnTimeout) < Timer.getFPGATimestamp() ) {
            // after 2 second, stop command
            System.out.println("turn timed out");
            return true;
        }
        return complete;
    }
}