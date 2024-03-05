// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoDropNoteAmpSeqCommand extends SequentialCommandGroup {

  Command cmd;

  public AutoDropNoteAmpSeqCommand(PhotonCamera photonCamera, 
        SwerveSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    
        Pose2d initialRobotPos2d = poseProvider.get();
        drivetrainSubsystem.swerveOdometry.resetPosition(drivetrainSubsystem.getYaw(), drivetrainSubsystem.getModulePositions(), 
        initialRobotPos2d);

        //Pose2d targetPose2d = new Pose2d(1,1,swerve.getYaw());
        //Pose2d targetPose2d = new Pose2d(1,0,  Rotation2d.fromDegrees(0+ drivetrainSubsystem.getYaw().getDegrees())   );
        Transform2d robotToGoal = new Transform2d(-0.5,-0.5, initialRobotPos2d.getRotation()) ; // has issue with only y change, not moving

        System.out.println("current pose = "+initialRobotPos2d);
        //System.out.println("target pose = "+targetPose2d);
        
        cmd = new AutoDriveToTargetPoseCommand(robotToGoal ,drivetrainSubsystem, drivetrainSubsystem::getPose);

      addCommands(
      
        new ParallelCommandGroup(
          new WaitUntilCommand(1)
          // mimic spinning up shooter 
          ,
          new  SequentialCommandGroup (
            new ChaseTagCommand(photonCamera, drivetrainSubsystem, poseProvider),
            new WaitCommand(2),
            cmd

          )
        )
        //.until(() -> !RobotContainer.intakeRollers.hasGamePiece() || RobotContainer.oi.intakeUpButton.getAsBoolean())

      );
  }


   
}
