// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.Commands.Autos;

//import edu.wpi.first.math.controller.PIDController;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Commands.Shoot;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter;

public class OnePieceLeft extends Command {

//CommandSwerveDrivetrain m_drive;
//Shooter m_shooter; 

  public OnePieceLeft(CommandSwerveDrivetrain drive, Shooter shooter) {
    
//this.m_drive = drive;
//this.m_shooter = shooter;

//addCommands(new Shoot(this.m_shooter).withTimeout(.5),
//new ProfiledDrivetoPosition(this.m_drive, 5.5, 3).withTimeout(5));



  }

  //private void addCommands(ParallelRaceGroup withTimeout, ParallelRaceGroup withTimeout2) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'addCommands'");
  }
//  }


/* If we are starting from the left with a pre-loaded piece
  drive while the conveyor passes the note
  accomodate for the weight and slow down
  when stopped and alligned, run pusher and shooter to get a point at the speaker

  initialize - reset encoder values

  execute - drive parellel (position) with conveyor (time control)
   Add a coast for drive train so it doesn't tip
   limelight check position

   if true (run pusher parallel with shooter)
   if false (toggle to the center of the april tag) loop back to true

   end - motors off on every subsystem

   or we can make seperate pieces of an auton and connect them all for what we
  need in robot container with parameters of different starting and ending points
 */
