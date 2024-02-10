package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class TurnToAngleCommand extends Command {

    private final SwerveSubsystem m_robotDrive;
    private boolean complete = false;
    private double angle;
    private double gyro_angle;
    private Timer timer = new Timer();
    private double timeout;
    public TurnToAngleCommand(SwerveSubsystem subsystem, double degrees, double timeoutS){
        m_robotDrive = subsystem;
        angle = degrees;
        timeout = timeoutS;
        //System.out.println("constructor  angle = "+ angle);
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        complete = false;
        gyro_angle = angle+m_robotDrive.getYaw().getDegrees(); // need start from current angle + degrees
        
        //System.out.println("turn start, gyro_angle = "+gyro_angle);
    }
    
    @Override
    public void execute(){
         double gyroAngle = m_robotDrive.getYaw().getDegrees();

        final double kP = 0.028;
        SmartDashboard.putNumber("gyroAngle", gyroAngle);
        //System.out.println("gyroAngle = "+gyroAngle+",  target gyro_angle = "+ gyro_angle);
        
    
        //double err = angle - gyroAngle;
        double err = gyro_angle - gyroAngle;
        double speed = MathUtil.clamp(err * kP, -Constants.Swerve.maxAngularVelocity*0.32, Constants.Swerve.maxAngularVelocity*0.32);
    
        if (Math.abs(err) > 2 && timer.get() < timeout) {
            m_robotDrive.drive(new Translation2d(0,0), speed, false, true);
        } else {
            complete = true;
           // System.out.println("turn complete");
        } 
    } 

    @Override
    public void end(boolean inturrupted){
        m_robotDrive.drive(new Translation2d(0,0), 0, false, true);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return complete;
    }
}