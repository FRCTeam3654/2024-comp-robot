// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropNoteAmpSeqCommand extends SequentialCommandGroup {



  public DropNoteAmpSeqCommand() {
    
      addCommands(
      
        new ParallelCommandGroup(
          new InstantCommand(() -> 
                RobotContainer.wrist.goToPositionBySmartMotion(-9.5)).withTimeout(4)
          ,
          new  SequentialCommandGroup (
                new WaitCommand(1),
                new ArmSmartMotion(3),
                new IntakeCommand(2)
          )
        )

      );
  }


   
}
